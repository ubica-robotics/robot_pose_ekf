/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <robot_pose_ekf/odom_estimation_node.hpp>
#include <ubica_rclcpp_utils/params.hpp>
#include <tf2_ros/create_timer_ros.h>


using namespace MatrixWrapper;
using namespace std;
using namespace rclcpp;
using namespace tf2;
using namespace std::placeholders;

static const double EPS = 1e-5;


//#define __EKF_DEBUG_FILE__

namespace estimation
{
  // constructor
  OdomEstimationNode::OdomEstimationNode(rclcpp::Node::SharedPtr nh)
    : nh_(nh), 
      logger_(nh->get_logger()),
      odom_active_(false),
      imu_active_(false),
      vo_active_(false),
      gps_active_(false),
      odom_initializing_(false),
      imu_initializing_(false),
      vo_initializing_(false),
      gps_initializing_(false),
      odom_covariance_(6),
      imu_covariance_(3),
      vo_covariance_(6),
      gps_covariance_(3),
      odom_callback_counter_(0),
      imu_callback_counter_(0),
      vo_callback_counter_(0),
      gps_callback_counter_(0),
      ekf_sent_counter_(0)
  {

    // paramters
    output_frame_ = ubica_rclcpp_utils::declare_and_get_param(nh_, "output_frame", std::string("odom_combined"));
    base_footprint_frame_ = ubica_rclcpp_utils::declare_and_get_param(nh_, "base_footprint_frame", std::string("base_footprint"));
    timeout_ = ubica_rclcpp_utils::declare_and_get_param(nh_, "sensor_timeout", 1.0);
    odom_used_ = ubica_rclcpp_utils::declare_and_get_param(nh_, "odom_used", true);
    imu_used_ = ubica_rclcpp_utils::declare_and_get_param(nh_, "imu_used", true);
    vo_used_ = ubica_rclcpp_utils::declare_and_get_param(nh_, "vo_used", true);
    gps_used_ = ubica_rclcpp_utils::declare_and_get_param(nh_, "gps_used", false);
    debug_ = ubica_rclcpp_utils::declare_and_get_param(nh_, "debug", false);
    self_diagnose_ = ubica_rclcpp_utils::declare_and_get_param(nh_, "self_diagnose", false);
    frame_prefix_  = ubica_rclcpp_utils::declare_and_get_param(nh_, "frame_prefix", std::string(""));
    double freq = ubica_rclcpp_utils::declare_and_get_param(nh_, "freq", 30.0);
    use_sim_time_ = nh_->get_parameter( "use_sim_time" ).as_bool();

    output_frame_ = frame_prefix_ + output_frame_;
    base_footprint_frame_ = frame_prefix_ + base_footprint_frame_;

    RCLCPP_INFO_STREAM(logger_, "output frame: " << output_frame_);
    RCLCPP_INFO_STREAM(logger_, "base frame: " << base_footprint_frame_);

    // set output frame and base frame names in OdomEstimation filter
    // so that user-defined tf frames are respected
    my_filter_.setOutputFrame(output_frame_);
    my_filter_.setBaseFootprintFrame(base_footprint_frame_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
    auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
      nh_->get_node_base_interface(), nh_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(cti);
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*nh_);

    timer_ = rclcpp::create_timer(nh_, nh_->get_clock(), durationFromSec(1.0/max(freq,1.0)), std::bind(&OdomEstimationNode::spin, this));
    
    // advertise our estimation
    pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/odom_combined", 10);

    // initialize
    filter_stamp_ = nh_->now();

    // subscribe to odom messages
    if (odom_used_){
      RCLCPP_DEBUG(logger_, "Odom sensor can be used");
      odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&OdomEstimationNode::odomCallback, this, _1));
    }
    else RCLCPP_DEBUG(logger_, "Odom sensor will NOT be used");

    // subscribe to imu messages
    if (imu_used_){
      RCLCPP_DEBUG(logger_, "Imu sensor can be used");
      imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>("imu_data", 10, std::bind(&OdomEstimationNode::imuCallback, this, _1));
    }
    else RCLCPP_DEBUG(logger_, "Imu sensor will NOT be used");

    // subscribe to vo messages
    if (vo_used_){
      RCLCPP_DEBUG(logger_, "VO sensor can be used");
      vo_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>("vo", 10, std::bind(&OdomEstimationNode::voCallback, this, _1));
    }
    else RCLCPP_DEBUG(logger_, "VO sensor will NOT be used");

    if (gps_used_){
      RCLCPP_DEBUG(logger_, "GPS sensor can be used");
      gps_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>("gps", 10, std::bind(&OdomEstimationNode::gpsCallback, this, _1));
    }
    else RCLCPP_DEBUG(logger_, "GPS sensor will NOT be used");


    // publish state service
    state_srv_ = nh_->create_service<robot_pose_ekf::srv::GetStatus>("~/get_status", std::bind(&OdomEstimationNode::getStatus, this, _1, _2));


    if (debug_){
      // open files for debugging
      odom_file_.open("/tmp/odom_file.txt");
      imu_file_.open("/tmp/imu_file.txt");
      vo_file_.open("/tmp/vo_file.txt");
      gps_file_.open("/tmp/gps_file.txt");
      corr_file_.open("/tmp/corr_file.txt");

  
    }
  }


  // destructor
  OdomEstimationNode::~OdomEstimationNode(){

    if (debug_){
      // close files for debugging
      odom_file_.close();
      imu_file_.close();
      gps_file_.close();
      vo_file_.close();
      corr_file_.close();
    }
  }


  // callback function for odom data
  void OdomEstimationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    odom_callback_counter_++;

    RCLCPP_DEBUG(logger_, "Odom callback at time %f ", nh_->now().seconds());
    assert(odom_used_);

    // receive data 
    odom_stamp_ = odom->header.stamp;

    odom_time_  = nh_->now();

    Quaternion q;
    tf2::fromMsg(odom->pose.pose.orientation, q);
    odom_meas_  = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0));
    for (unsigned int i=0; i<6; i++) {
      for (unsigned int j=0; j<6; j++) {
        odom_covariance_(i+1, j+1) = odom->pose.covariance[6*i+j];
      }
    }

    geometry_msgs::msg::TransformStamped tf_to_msg;
    tf_to_msg.header.stamp = tf2_ros::toMsg(tf2_ros::fromRclcpp(odom_stamp_));
    tf_to_msg.header.frame_id = base_footprint_frame_;
    tf_to_msg.child_frame_id = "wheelodom";
    auto tf = odom_meas_.inverse();
    tf_to_msg.transform.translation.x = tf.getOrigin().getX();
    tf_to_msg.transform.translation.y = tf.getOrigin().getY();
    tf_to_msg.transform.translation.z = tf.getOrigin().getZ();
    tf_to_msg.transform.rotation = tf2::toMsg(tf.getRotation());
    my_filter_.addMeasurement(tf_to_msg, odom_covariance_);
    
    // activate odom
    if (!odom_active_) {
      if (!odom_initializing_){
	odom_initializing_ = true;
	odom_init_stamp_ = odom_stamp_;
	RCLCPP_INFO(logger_, "Initializing Odom sensor");      
      }
      if ( filter_stamp_ >= odom_init_stamp_){
	odom_active_ = true;
	odom_initializing_ = false;
	RCLCPP_INFO(logger_, "Odom sensor activated");      
      }
      else RCLCPP_INFO(logger_, "Waiting to activate Odom, because Odom measurements are still %f sec in the future.", 
		    (odom_init_stamp_ - filter_stamp_).seconds());
    }
    
    if (debug_){
      // write to file
      double tmp, yaw;
      odom_meas_.getBasis().getEulerYPR(yaw, tmp, tmp);
      odom_file_<< fixed <<setprecision(5) << nh_->now().seconds() << " " << odom_meas_.getOrigin().x() << " " << odom_meas_.getOrigin().y() << "  " << yaw << "  " << endl;
    }
  }


  // callback function for imu data
  void OdomEstimationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
  {
    imu_callback_counter_++;

    assert(imu_used_);

    // receive data 
    imu_stamp_ = imu->header.stamp;

    tf2::Quaternion orientation;
    tf2::fromMsg(imu->orientation, orientation);
    imu_meas_ = tf2::Transform(orientation, tf2::Vector3(0,0,0));
    for (unsigned int i=0; i<3; i++){
      for (unsigned int j=0; j<3; j++){
        imu_covariance_(i+1, j+1) = imu->orientation_covariance[3*i+j];
      }
    }

    auto tf_future = tf_buffer_->waitForTransform(base_footprint_frame_, imu->header.frame_id, imu_stamp_, rclcpp::Duration::from_seconds(0), [](auto &) {});
    auto status = tf_future.wait_for(durationFromSec(0.5));
    if (status != std::future_status::ready) {
      // warn when imu was already activated, not when imu is not active yet
      if (imu_active_) {
          RCLCPP_ERROR(logger_, "Could not transform imu message from %s to %s", imu->header.frame_id.c_str(), 
                       base_footprint_frame_.c_str());
      }
      else if (my_filter_.isInitialized()) {
          RCLCPP_WARN(logger_, "Could not transform imu message from %s to %s. Imu will not be activated yet.", 
                      imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
          }
      else {
          RCLCPP_DEBUG(logger_, "Could not transform imu message from %s to %s. Imu will not be activated yet.", 
                       imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      }
      return;
    }

    geometry_msgs::msg::TransformStamped base_imu_offset;
    base_imu_offset = tf_buffer_->lookupTransform(base_footprint_frame_, imu->header.frame_id, imu_stamp_, durationFromSec(0.5));
    
    tf2::Stamped<tf2::Transform> tf2;
    tf2::convert(base_imu_offset, tf2);
    imu_meas_ = imu_meas_ * tf2;

    imu_time_  = nh_->now();

    // manually set covariance untile imu sends covariance
    if (imu_covariance_(1,1) == 0.0){
      SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
      measNoiseImu_Cov(1,1) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(2,2) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(3,3) = pow(0.00017,2);  // = 0.01 degrees / sec
      imu_covariance_ = measNoiseImu_Cov;
    }

    geometry_msgs::msg::TransformStamped tf_to_msg;
    tf_to_msg.header.stamp = tf2_ros::toMsg(tf2_ros::fromRclcpp(imu_stamp_));
    tf_to_msg.header.frame_id = base_footprint_frame_;
    tf_to_msg.child_frame_id = "imu";
    auto tf = imu_meas_.inverse();
    tf_to_msg.transform.translation.x = tf.getOrigin().getX();
    tf_to_msg.transform.translation.y = tf.getOrigin().getY();
    tf_to_msg.transform.translation.z = tf.getOrigin().getZ();
    tf_to_msg.transform.rotation = tf2::toMsg(tf.getRotation());

    my_filter_.addMeasurement(tf_to_msg, imu_covariance_);
    
    // activate imu
    if (!imu_active_) {
      if (!imu_initializing_){
	imu_initializing_ = true;
	imu_init_stamp_ = imu_stamp_;
	RCLCPP_INFO(logger_, "Initializing Imu sensor");      
      }
      if ( filter_stamp_ >= imu_init_stamp_){
	imu_active_ = true;
	imu_initializing_ = false;
	RCLCPP_INFO(logger_, "Imu sensor activated");      
      }
      else RCLCPP_DEBUG(logger_, "Waiting to activate IMU, because IMU measurements are still %f sec in the future.", 
		    (imu_init_stamp_ - filter_stamp_).seconds());
    }
    
    if (debug_){
      // write to file
      double tmp, yaw;
      imu_meas_.getBasis().getEulerYPR(yaw, tmp, tmp); 
      imu_file_ <<fixed<<setprecision(5)<<nh_->now().seconds()<<" "<< yaw << endl;
    }
  }


  // callback function for VO data
  void OdomEstimationNode::voCallback(const nav_msgs::msg::Odometry::SharedPtr vo)
  {
    vo_callback_counter_++;

    assert(vo_used_);

    // get data
    vo_stamp_ = vo->header.stamp;
    vo_time_  = nh_->now();
    tf2::fromMsg(vo->pose.pose, vo_meas_);
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        vo_covariance_(i+1, j+1) = vo->pose.covariance[6*i+j];

    geometry_msgs::msg::TransformStamped tf_to_msg;
    tf_to_msg.header.stamp = tf2_ros::toMsg(tf2_ros::fromRclcpp(vo_stamp_));
    tf_to_msg.header.frame_id = base_footprint_frame_;
    tf_to_msg.child_frame_id = "vo";
    auto tf = vo_meas_.inverse();
    tf_to_msg.transform.translation.x = tf.getOrigin().getX();
    tf_to_msg.transform.translation.y = tf.getOrigin().getY();
    tf_to_msg.transform.translation.z = tf.getOrigin().getZ();
    tf_to_msg.transform.rotation = tf2::toMsg(tf.getRotation());

    my_filter_.addMeasurement(tf_to_msg);
    
    // activate vo
    if (!vo_active_) {
      if (!vo_initializing_){
	vo_initializing_ = true;
	vo_init_stamp_ = vo_stamp_;
	RCLCPP_INFO(logger_, "Initializing Vo sensor");      
      }
      if (filter_stamp_ >= vo_init_stamp_){
	vo_active_ = true;
	vo_initializing_ = false;
	RCLCPP_INFO(logger_, "Vo sensor activated");      
      }
      else RCLCPP_DEBUG(logger_, "Waiting to activate VO, because VO measurements are still %f sec in the future.", 
		    (vo_init_stamp_ - filter_stamp_).seconds());
    }
    
    if (debug_){
      // write to file
      double Rx, Ry, Rz;
      vo_meas_.getBasis().getEulerYPR(Rz, Ry, Rx);
      vo_file_ <<fixed<<setprecision(5)<<nh_->now().seconds()<<" "<< vo_meas_.getOrigin().x() << " " << vo_meas_.getOrigin().y() << " " << vo_meas_.getOrigin().z() << " "
               << Rx << " " << Ry << " " << Rz << endl;
    }
  }


  void OdomEstimationNode::gpsCallback(const nav_msgs::msg::Odometry::SharedPtr gps)
  {
    gps_callback_counter_++;

    assert(gps_used_);

    // get data
    gps_stamp_ = gps->header.stamp;
    gps_time_  = nh_->now();
    geometry_msgs::msg::PoseWithCovariance gps_pose = gps->pose;
    if (isnan(gps_pose.pose.position.z)){
      // if we have no linear z component in the GPS message, set it to 0 so that we can still get a transform via `tf
      // (which does not like "NaN" values)
      gps_pose.pose.position.z = 0;
      // set the covariance for the linear z component very high so we just ignore it
      gps_pose.covariance[6*2 + 2] = std::numeric_limits<double>::max();
    }
    tf2::fromMsg(gps_pose.pose, gps_meas_);
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        gps_covariance_(i+1, j+1) = gps_pose.covariance[6*i+j];

    geometry_msgs::msg::TransformStamped tf_to_msg;
    tf_to_msg.header.stamp = tf2_ros::toMsg(tf2_ros::fromRclcpp(gps_stamp_));
    tf_to_msg.header.frame_id = base_footprint_frame_;
    tf_to_msg.child_frame_id = "gps";
    auto tf = gps_meas_.inverse();
    tf_to_msg.transform.translation.x = tf.getOrigin().getX();
    tf_to_msg.transform.translation.y = tf.getOrigin().getY();
    tf_to_msg.transform.translation.z = tf.getOrigin().getZ();
    tf_to_msg.transform.rotation = tf2::toMsg(tf.getRotation());

    my_filter_.addMeasurement(tf_to_msg);
    
    // activate gps
    if (!gps_active_) {
      if (!gps_initializing_){
	    gps_initializing_ = true;
	    gps_init_stamp_ = gps_stamp_;
	    RCLCPP_INFO(logger_, "Initializing GPS sensor");      
      }
      if (filter_stamp_ >= gps_init_stamp_){
	    gps_active_ = true;
	    gps_initializing_ = false;
	    RCLCPP_INFO(logger_, "GPS sensor activated");      
      }
      else RCLCPP_DEBUG(logger_, "Waiting to activate GPS, because GPS measurements are still %f sec in the future.", 
		    (gps_init_stamp_ - filter_stamp_).seconds());
    }
    
    if (debug_){
      // write to file
      gps_file_ <<fixed<<setprecision(5)<<nh_->now().seconds()<<" "<< gps_meas_.getOrigin().x() << " " << gps_meas_.getOrigin().y() << " " << gps_meas_.getOrigin().z() <<endl;
    }
  }


  // filter loop
  void OdomEstimationNode::spin()
  {
    RCLCPP_DEBUG(logger_, "Spin function at time %f", nh_->now().seconds());

    // check for timing problems
    if ( (odom_initializing_ || odom_active_) && (imu_initializing_ || imu_active_) ){
      double diff = fabs( rclcpp::Duration(odom_stamp_ - imu_stamp_).seconds() );
      if (diff > 1.0) RCLCPP_ERROR(logger_, "Timestamps of odometry and imu are %f seconds apart.", diff);
    }
    
    // initial value for filter stamp; keep this stamp when no sensors are active
    filter_stamp_ = nh_->now();
    
    // check which sensors are still active
    if ((odom_active_ || odom_initializing_) && 
        (nh_->now() - odom_time_).seconds() > timeout_){
      odom_active_ = false; odom_initializing_ = false;
      RCLCPP_DEBUG(logger_, "Odom sensor not active any more");
    }
    if ((imu_active_ || imu_initializing_) && 
        (nh_->now() - imu_time_).seconds() > timeout_){
      imu_active_ = false;  imu_initializing_ = false;
      RCLCPP_DEBUG(logger_, "Imu sensor not active any more");
    }
    if ((vo_active_ || vo_initializing_) && 
        (nh_->now() - vo_time_).seconds() > timeout_){
      vo_active_ = false;  vo_initializing_ = false;
      RCLCPP_DEBUG(logger_, "VO sensor not active any more");
    }

    if ((gps_active_ || gps_initializing_) && 
        (nh_->now() - gps_time_).seconds() > timeout_){
      gps_active_ = false;  gps_initializing_ = false;
      RCLCPP_DEBUG(logger_, "GPS sensor not active any more");
    }

    // only update filter when one of the sensors is active
    if (odom_active_ || imu_active_ || vo_active_ || gps_active_){
      
      // update filter at time where all sensor measurements are available
      if (odom_active_)  filter_stamp_ = min(filter_stamp_, odom_stamp_);
      if (imu_active_)   filter_stamp_ = min(filter_stamp_, imu_stamp_);
      if (vo_active_)    filter_stamp_ = min(filter_stamp_, vo_stamp_);
      if (gps_active_)  filter_stamp_ = min(filter_stamp_, gps_stamp_);

      // update filter
      if ( my_filter_.isInitialized() )  {
        bool diagnostics = true;
        if (my_filter_.update(odom_active_, imu_active_,gps_active_, vo_active_,  filter_stamp_, diagnostics)){
          // output most recent estimate and relative covariance
          my_filter_.getEstimate(output_);
          pose_pub_->publish(output_);
          ekf_sent_counter_++;
          
          // broadcast most recent estimate to TransformArray
          geometry_msgs::msg::TransformStamped tmp;
          my_filter_.getEstimate(rclcpp::Time(), tmp);
          if(!vo_active_ && !gps_active_)
            tmp.transform.translation.z = 0.0;

          odom_broadcaster_->sendTransform(tmp);
          
          if (debug_){
            // write to file
            ColumnVector estimate; 
            my_filter_.getEstimate(estimate);
            corr_file_ << fixed << setprecision(5)<<nh_->now().seconds()<<" ";
            
            for (unsigned int i=1; i<=6; i++)
            corr_file_ << estimate(i) << " ";
            corr_file_ << endl;
          }
        }
        if (self_diagnose_ && !diagnostics)
          RCLCPP_WARN(logger_, "Robot pose ekf diagnostics discovered a potential problem");
      }

      // initialize filer with odometry frame
      if (imu_active_ && gps_active_ && !my_filter_.isInitialized()) {
	Quaternion q = imu_meas_.getRotation();
        Vector3 p = gps_meas_.getOrigin();
        Transform init_meas_ = Transform(q, p);
        my_filter_.initialize(init_meas_, gps_stamp_);
        RCLCPP_INFO(logger_, "Kalman filter initialized with gps and imu measurement");
      }	
      else if ( odom_active_ && gps_active_ && !my_filter_.isInitialized()) {
	Quaternion q = odom_meas_.getRotation();
        Vector3 p = gps_meas_.getOrigin();
        Transform init_meas_ = Transform(q, p);
        my_filter_.initialize(init_meas_, gps_stamp_);
        RCLCPP_INFO(logger_, "Kalman filter initialized with gps and odometry measurement");
      }
      else if ( vo_active_ && gps_active_ && !my_filter_.isInitialized()) {
	Quaternion q = vo_meas_.getRotation();
        Vector3 p = gps_meas_.getOrigin();
        Transform init_meas_ = Transform(q, p);
        my_filter_.initialize(init_meas_, gps_stamp_);
        RCLCPP_INFO(logger_, "Kalman filter initialized with gps and visual odometry measurement");
      }
      else if ( odom_active_  && !gps_used_ && !my_filter_.isInitialized()){
        my_filter_.initialize(odom_meas_, odom_stamp_);
        RCLCPP_INFO(logger_, "Kalman filter initialized with odom measurement");
      }
      else if ( vo_active_ && !gps_used_ && !my_filter_.isInitialized()){
        my_filter_.initialize(vo_meas_, vo_stamp_);
        RCLCPP_INFO(logger_, "Kalman filter initialized with vo measurement");
      }
    }
  }


  bool OdomEstimationNode::getStatus(const robot_pose_ekf::srv::GetStatus::Request::SharedPtr req, 
                                     robot_pose_ekf::srv::GetStatus::Response::SharedPtr resp)
  {
      (void)req;
      stringstream ss;
      ss << "Input:" << endl;
      ss << " * Odometry sensor" << endl;
      ss << "   - is "; if (!odom_used_) ss << "NOT "; ss << "used" << endl;
      ss << "   - is "; if (!odom_active_) ss << "NOT "; ss << "active" << endl;
      ss << "   - received " << odom_callback_counter_ << " messages" << endl;
      ss << "   - listens to topic " << odom_sub_->get_topic_name() << endl;
      ss << " * IMU sensor" << endl;
      ss << "   - is "; if (!imu_used_) ss << "NOT "; ss << "used" << endl;
      ss << "   - is "; if (!imu_active_) ss << "NOT "; ss << "active" << endl;
      ss << "   - received " << imu_callback_counter_ << " messages" << endl;
      ss << "   - listens to topic " << imu_sub_->get_topic_name() << endl;
      ss << " * Visual Odometry sensor" << endl;
      ss << "   - is "; if (!vo_used_) ss << "NOT "; ss << "used" << endl;
      ss << "   - is "; if (!vo_active_) ss << "NOT "; ss << "active" << endl;
      ss << "   - received " << vo_callback_counter_ << " messages" << endl;
      ss << "   - listens to topic " << vo_sub_->get_topic_name() << endl;
      ss << " * GPS sensor" << endl;
      ss << "   - is "; if (!gps_used_) ss << "NOT "; ss << "used" << endl;
      ss << "   - is "; if (!gps_active_) ss << "NOT "; ss << "active" << endl;
      ss << "   - received " << gps_callback_counter_ << " messages" << endl;
      ss << "   - listens to topic " << gps_sub_->get_topic_name() << endl;
      ss << "Output:" << endl;
      ss << " * Robot pose ekf filter" << endl;
      ss << "   - is "; if (!my_filter_.isInitialized()) ss << "NOT "; ss << "active" << endl;
      ss << "   - sent " << ekf_sent_counter_ << " messages" << endl;
      ss << "   - pulishes on topics " << pose_pub_->get_topic_name() << " and /tf" << endl;
      resp->status = ss.str();
      return true;
  }


} // namespace






// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("robot_pose_ekf");

  // create filter class
  OdomEstimationNode my_filter_node(node);

  while (rclcpp::ok()){
      rclcpp::spin(node);
  }
  
  return 0;
}
