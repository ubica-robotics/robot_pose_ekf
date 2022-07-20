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

#include <robot_pose_ekf/odom_estimation.hpp>

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf2;
using namespace std;
using namespace rclcpp;


namespace estimation
{
  // constructor
  OdomEstimation::OdomEstimation():
    prior_(NULL),
    filter_(NULL),
    filter_initialized_(false),
    odom_initialized_(false),
    imu_initialized_(false),
    vo_initialized_(false),
    gps_initialized_(false),
    output_frame_(std::string("odom_combined")),
    base_footprint_frame_(std::string("base_footprint")),
    logger_(rclcpp::get_logger("odom_logger"))
  {
    // create SYSTEM MODEL
    ColumnVector sysNoise_Mu(6);  sysNoise_Mu = 0;
    SymmetricMatrix sysNoise_Cov(6); sysNoise_Cov = 0;
    for (unsigned int i=1; i<=6; i++) sysNoise_Cov(i,i) = pow(1000.0,2);
    Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    sys_pdf_   = new NonLinearAnalyticConditionalGaussianOdo(system_Uncertainty);
    sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

    // create MEASUREMENT MODEL ODOM
    ColumnVector measNoiseOdom_Mu(6);  measNoiseOdom_Mu = 0;
    SymmetricMatrix measNoiseOdom_Cov(6);  measNoiseOdom_Cov = 0;
    for (unsigned int i=1; i<=6; i++) measNoiseOdom_Cov(i,i) = 1;
    Gaussian measurement_Uncertainty_Odom(measNoiseOdom_Mu, measNoiseOdom_Cov);
    Matrix Hodom(6,6);  Hodom = 0;
    Hodom(1,1) = 1;    Hodom(2,2) = 1;    Hodom(6,6) = 1;
    odom_meas_pdf_   = new LinearAnalyticConditionalGaussian(Hodom, measurement_Uncertainty_Odom);
    odom_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(odom_meas_pdf_);

    // create MEASUREMENT MODEL IMU
    ColumnVector measNoiseImu_Mu(3);  measNoiseImu_Mu = 0;
    SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
    for (unsigned int i=1; i<=3; i++) measNoiseImu_Cov(i,i) = 1;
    Gaussian measurement_Uncertainty_Imu(measNoiseImu_Mu, measNoiseImu_Cov);
    Matrix Himu(3,6);  Himu = 0;
    Himu(1,4) = 1;    Himu(2,5) = 1;    Himu(3,6) = 1;
    imu_meas_pdf_   = new LinearAnalyticConditionalGaussian(Himu, measurement_Uncertainty_Imu);
    imu_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(imu_meas_pdf_);

    // create MEASUREMENT MODEL VO
    ColumnVector measNoiseVo_Mu(6);  measNoiseVo_Mu = 0;
    SymmetricMatrix measNoiseVo_Cov(6);  measNoiseVo_Cov = 0;
    for (unsigned int i=1; i<=6; i++) measNoiseVo_Cov(i,i) = 1;
    Gaussian measurement_Uncertainty_Vo(measNoiseVo_Mu, measNoiseVo_Cov);
    Matrix Hvo(6,6);  Hvo = 0;
    Hvo(1,1) = 1;    Hvo(2,2) = 1;    Hvo(3,3) = 1;    Hvo(4,4) = 1;    Hvo(5,5) = 1;    Hvo(6,6) = 1;
    vo_meas_pdf_   = new LinearAnalyticConditionalGaussian(Hvo, measurement_Uncertainty_Vo);
    vo_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(vo_meas_pdf_);

    // create MEASUREMENT MODEL GPS
    ColumnVector measNoiseGps_Mu(3);  measNoiseGps_Mu = 0;
    SymmetricMatrix measNoiseGps_Cov(3);  measNoiseGps_Cov = 0;
    for (unsigned int i=1; i<=3; i++) measNoiseGps_Cov(i,i) = 1;
    Gaussian measurement_Uncertainty_GPS(measNoiseGps_Mu, measNoiseGps_Cov);
    Matrix Hgps(3,6);  Hgps = 0;
    Hgps(1,1) = 1;    Hgps(2,2) = 1;    Hgps(3,3) = 1;    
    gps_meas_pdf_   = new LinearAnalyticConditionalGaussian(Hgps, measurement_Uncertainty_GPS);
    gps_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(gps_meas_pdf_);

    rclcpp::Clock::SharedPtr clock; 
    clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    transformer_ = std::make_unique<tf2_ros::Buffer>(clock);
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*transformer_);
  }



  // destructor
  OdomEstimation::~OdomEstimation(){
    if (filter_) delete filter_;
    if (prior_)  delete prior_;
    delete odom_meas_model_;
    delete odom_meas_pdf_;
    delete imu_meas_model_;
    delete imu_meas_pdf_;
    delete vo_meas_model_;
    delete vo_meas_pdf_;
    delete gps_meas_model_;
    delete gps_meas_pdf_;
    delete sys_pdf_;
    delete sys_model_;
  }


  // initialize prior density of filter 
  void OdomEstimation::initialize(const Transform& prior, const Time& time)
  {
    // set prior of filter
    ColumnVector prior_Mu(6); 
    decomposeTransform(prior, prior_Mu(1), prior_Mu(2), prior_Mu(3), prior_Mu(4), prior_Mu(5), prior_Mu(6));
    SymmetricMatrix prior_Cov(6); 
    for (unsigned int i=1; i<=6; i++) {
      for (unsigned int j=1; j<=6; j++){
	if (i==j)  prior_Cov(i,j) = pow(0.001,2);
	else prior_Cov(i,j) = 0;
      }
    }
    prior_  = new Gaussian(prior_Mu,prior_Cov);
    filter_ = new ExtendedKalmanFilter(prior_);

    geometry_msgs::msg::TransformStamped tf_to_msg;
    tf_to_msg.header.stamp = tf2_ros::toMsg(tf2_ros::fromRclcpp(time));
    tf_to_msg.header.frame_id = output_frame_;
    tf_to_msg.child_frame_id = base_footprint_frame_;
    tf_to_msg.transform.translation.x = prior.getOrigin().getX();
    tf_to_msg.transform.translation.y = prior.getOrigin().getY();
    tf_to_msg.transform.translation.z = prior.getOrigin().getZ();
    tf_to_msg.transform.rotation = tf2::toMsg(prior.getRotation());

    // remember prior
    addMeasurement(tf_to_msg);
    filter_estimate_old_vec_ = prior_Mu;
    filter_estimate_old_ = prior;
    filter_time_old_     = time;

    // filter initialized
    filter_initialized_ = true;
  }





  // update filter
  bool OdomEstimation::update(bool odom_active, bool imu_active, bool gps_active, bool vo_active, const Time&  filter_time, bool& diagnostics_res)
  {
    // only update filter when it is initialized
    if (!filter_initialized_){
      RCLCPP_INFO(logger_, "Cannot update filter when filter was not initialized first.");
      return false;
    }

    // only update filter for time later than current filter time
    double dt = (filter_time - filter_time_old_).seconds();
    if (dt == 0) return false;
    if (dt <  0){
      RCLCPP_INFO(logger_, "Will not update robot pose with time %f sec in the past.", dt);
      return false;
    }
    RCLCPP_DEBUG(logger_, "Update filter at time %f with dt %f", filter_time.seconds(), dt);


    // system update filter
    // --------------------
    // for now only add system noise
    ColumnVector vel_desi(2); vel_desi = 0;
    filter_->Update(sys_model_, vel_desi);
    
    // process odom measurement
    // ------------------------
    RCLCPP_DEBUG(logger_, "Process odom meas");
    if (odom_active){
      if (!transformer_->canTransform(base_footprint_frame_,"wheelodom", filter_time)){
        RCLCPP_ERROR(logger_, "filter time older than odom message buffer");
        return false;
      }
      odom_meas_ = transformer_->lookupTransform("wheelodom", base_footprint_frame_, filter_time);

      if (odom_initialized_){
        tf2::Quaternion q;
        q.setRPY(0.0f, 0.0f, filter_estimate_old_vec_(6));
        tf2::Transform odom_meas_old_tf;
        tf2::fromMsg(odom_meas_old_.transform, odom_meas_old_tf);
        tf2::Transform odom_meas_tf;
        tf2::fromMsg(odom_meas_.transform, odom_meas_tf);
	
	// convert absolute odom measurements to relative odom measurements in horizontal plane
	Transform odom_rel_frame =  Transform(q, 
					      filter_estimate_old_.getOrigin()) * odom_meas_old_tf.inverse() * odom_meas_tf;
	ColumnVector odom_rel(6); 
	decomposeTransform(odom_rel_frame, odom_rel(1), odom_rel(2), odom_rel(3), odom_rel(4), odom_rel(5), odom_rel(6));
	angleOverflowCorrect(odom_rel(6), filter_estimate_old_vec_(6));
	// update filter
	odom_meas_pdf_->AdditiveNoiseSigmaSet(odom_covariance_ * pow(dt,2));
	RCLCPP_DEBUG(logger_, "Update filter with odom measurement %f %f %f %f %f %f", 
                  odom_rel(1), odom_rel(2), odom_rel(3), odom_rel(4), odom_rel(5), odom_rel(6));
	filter_->Update(odom_meas_model_, odom_rel);
	diagnostics_odom_rot_rel_ = odom_rel(6);
      }
      else{
	odom_initialized_ = true;
	diagnostics_odom_rot_rel_ = 0;
      }
      odom_meas_old_ = odom_meas_;
    }
    // sensor not active
    else odom_initialized_ = false;

    
    // process imu measurement
    // -----------------------
    if (imu_active){
      if (!transformer_->canTransform(base_footprint_frame_,"imu", filter_time)){
        RCLCPP_ERROR(logger_, "filter time older than imu message buffer");
        return false;
      }
      imu_meas_ = transformer_->lookupTransform("imu", base_footprint_frame_, filter_time);

      if (imu_initialized_){
        tf2::Transform imu_meas_old_tf;
        tf2::fromMsg(imu_meas_old_.transform, imu_meas_old_tf);
        tf2::Transform imu_meas_tf;
        tf2::fromMsg(imu_meas_.transform, imu_meas_tf);
	// convert absolute imu yaw measurement to relative imu yaw measurement 
	Transform imu_rel_frame =  filter_estimate_old_ * imu_meas_old_tf.inverse() * imu_meas_tf;
	ColumnVector imu_rel(3); double tmp;
	decomposeTransform(imu_rel_frame, tmp, tmp, tmp, tmp, tmp, imu_rel(3));
	decomposeTransform(imu_meas_,     tmp, tmp, tmp, imu_rel(1), imu_rel(2), tmp);
	angleOverflowCorrect(imu_rel(3), filter_estimate_old_vec_(6));
	diagnostics_imu_rot_rel_ = imu_rel(3);
	// update filter
	imu_meas_pdf_->AdditiveNoiseSigmaSet(imu_covariance_ * pow(dt,2));
	filter_->Update(imu_meas_model_,  imu_rel);
      }
      else{
	imu_initialized_ = true;
	diagnostics_imu_rot_rel_ = 0;
      }
      imu_meas_old_ = imu_meas_; 
    }
    // sensor not active
    else imu_initialized_ = false;
    
    
    
    // process vo measurement
    // ----------------------
    if (vo_active){
      if (!transformer_->canTransform(base_footprint_frame_,"vo", filter_time)){
        RCLCPP_ERROR(logger_, "filter time older than vo message buffer");
        return false;
      }
      vo_meas_ = transformer_->lookupTransform("vo", base_footprint_frame_, filter_time);
      if (vo_initialized_){
        tf2::Transform vo_meas_old_tf;
        tf2::fromMsg(vo_meas_old_.transform, vo_meas_old_tf);
        tf2::Transform vo_meas_tf;
        tf2::fromMsg(vo_meas_.transform, vo_meas_tf);
	// convert absolute vo measurements to relative vo measurements
	Transform vo_rel_frame =  filter_estimate_old_ * vo_meas_old_tf.inverse() * vo_meas_tf;
	ColumnVector vo_rel(6);
	decomposeTransform(vo_rel_frame, vo_rel(1),  vo_rel(2), vo_rel(3), vo_rel(4), vo_rel(5), vo_rel(6));
	angleOverflowCorrect(vo_rel(6), filter_estimate_old_vec_(6));
	// update filter
        vo_meas_pdf_->AdditiveNoiseSigmaSet(vo_covariance_ * pow(dt,2));
        filter_->Update(vo_meas_model_,  vo_rel);
      }
      else vo_initialized_ = true;
      vo_meas_old_ = vo_meas_;
    }
    // sensor not active
    else vo_initialized_ = false;
  


    // process gps measurement
    // ----------------------
    if (gps_active){
      if (!transformer_->canTransform(base_footprint_frame_,"gps", filter_time)){
        RCLCPP_ERROR(logger_, "filter time older than gps message buffer");
        return false;
      }
      gps_meas_ = transformer_->lookupTransform("gps", base_footprint_frame_, filter_time);
      if (gps_initialized_){
        gps_meas_pdf_->AdditiveNoiseSigmaSet(gps_covariance_ * pow(dt,2));
        ColumnVector gps_vec(3);
        double tmp;
        //Take gps as an absolute measurement, do not convert to relative measurement
        decomposeTransform(gps_meas_, gps_vec(1), gps_vec(2), gps_vec(3), tmp, tmp, tmp);
        filter_->Update(gps_meas_model_,  gps_vec);
      }
      else {
        gps_initialized_ = true;
        gps_meas_old_ = gps_meas_;
      }
    }
    // sensor not active
    else gps_initialized_ = false;

  
    
    // remember last estimate
    filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
    tf2::Quaternion q;
    q.setRPY(filter_estimate_old_vec_(4), filter_estimate_old_vec_(5), filter_estimate_old_vec_(6));
    filter_estimate_old_ = Transform(q,
				     Vector3(filter_estimate_old_vec_(1), filter_estimate_old_vec_(2), filter_estimate_old_vec_(3)));
    filter_time_old_ = filter_time;

    geometry_msgs::msg::TransformStamped tf_to_msg;
    tf_to_msg.header.stamp = tf2_ros::toMsg(tf2_ros::fromRclcpp(filter_time));
    tf_to_msg.header.frame_id = output_frame_;
    tf_to_msg.child_frame_id = base_footprint_frame_;
    tf_to_msg.transform.translation.x = filter_estimate_old_.getOrigin().getX();
    tf_to_msg.transform.translation.y = filter_estimate_old_.getOrigin().getY();
    tf_to_msg.transform.translation.z = filter_estimate_old_.getOrigin().getZ();
    tf_to_msg.transform.rotation = tf2::toMsg(filter_estimate_old_.getRotation());

    addMeasurement(tf_to_msg);
    
    // diagnostics
    diagnostics_res = true;
    if (odom_active && imu_active){
      double diagnostics = fabs(diagnostics_odom_rot_rel_ - diagnostics_imu_rot_rel_)/dt;
      if (diagnostics > 0.3 && dt > 0.01){
	diagnostics_res = false;
      }
    }

    return true;
  }

  void OdomEstimation::addMeasurement(const geometry_msgs::msg::TransformStamped& meas)
  {
    RCLCPP_DEBUG(logger_, "AddMeasurement from %s to %s:  (%f, %f, %f)  (%f, %f, %f, %f)",
              meas.header.frame_id.c_str(), meas.child_frame_id.c_str(),
              meas.transform.translation.x, meas.transform.translation.y, meas.transform.translation.z,
              meas.transform.rotation.x, meas.transform.rotation.y, 
              meas.transform.rotation.z, meas.transform.rotation.w);
    transformer_->setTransform(meas, "default_authority");
  }

  void OdomEstimation::addMeasurement(const geometry_msgs::msg::TransformStamped& meas, const MatrixWrapper::SymmetricMatrix& covar)
  {
    // check covariance
    for (unsigned int i=0; i<covar.rows(); i++){
      if (covar(i+1,i+1) == 0){
        RCLCPP_ERROR(logger_, "Covariance specified for measurement on topic %s is zero", meas.child_frame_id.c_str());
        return;
      }
    }
    // add measurements
    addMeasurement(meas);
    if (meas.child_frame_id == "wheelodom") odom_covariance_ = covar;
    else if (meas.child_frame_id == "imu")  imu_covariance_  = covar;
    else if (meas.child_frame_id == "vo")   vo_covariance_   = covar;
    else if (meas.child_frame_id == "gps")  gps_covariance_  = covar;
    else RCLCPP_ERROR(logger_, "Adding a measurement for an unknown sensor %s", meas.child_frame_id.c_str());
  }


  // get latest filter posterior as vector
  void OdomEstimation::getEstimate(MatrixWrapper::ColumnVector& estimate)
  {
    estimate = filter_estimate_old_vec_;
  }

  // get filter posterior at time 'time' as Transform
  void OdomEstimation::getEstimate(Time time, Transform& estimate)
  {
    geometry_msgs::msg::TransformStamped tmp;
    if (!transformer_->canTransform(base_footprint_frame_,output_frame_, time)){
      RCLCPP_ERROR(logger_, "Cannot get transform at time %f", time.seconds());
      return;
    }
    tmp = transformer_->lookupTransform(output_frame_, base_footprint_frame_, time);
    tf2::Transform est_tf;
    tf2::fromMsg(tmp.transform, est_tf);
    estimate = est_tf;
  }

  // get filter posterior at time 'time' as Stamped Transform
  void OdomEstimation::getEstimate(Time time, geometry_msgs::msg::TransformStamped& estimate)
  {
    if (!transformer_->canTransform(output_frame_, base_footprint_frame_, time)){
      RCLCPP_ERROR(logger_, "Cannot get transform at time %f", time.seconds());
      return;
    }
    estimate = transformer_->lookupTransform(output_frame_, base_footprint_frame_, time);
  }

  // get most recent filter posterior as PoseWithCovarianceStamped
  void OdomEstimation::getEstimate(geometry_msgs::msg::PoseWithCovarianceStamped& estimate)
  {
    // pose
    geometry_msgs::msg::TransformStamped tmp;
    if (!transformer_->canTransform(output_frame_, base_footprint_frame_, rclcpp::Time())){
      RCLCPP_ERROR(logger_, "Cannot get transform at time %f", 0.0);
      return;
    }
    tmp = transformer_->lookupTransform(output_frame_, base_footprint_frame_, rclcpp::Time());
    estimate.pose.pose.position.x = tmp.transform.translation.x;
    estimate.pose.pose.position.y = tmp.transform.translation.y;
    estimate.pose.pose.position.z = tmp.transform.translation.z;
    estimate.pose.pose.orientation = tmp.transform.rotation;

    // header
    estimate.header.stamp = tmp.header.stamp;
    estimate.header.frame_id = output_frame_;

    // covariance
    SymmetricMatrix covar =  filter_->PostGet()->CovarianceGet();
    for (unsigned int i=0; i<6; i++) {
      for (unsigned int j=0; j<6; j++) {
	estimate.pose.covariance[6*i+j] = covar(i+1,j+1);
      }
    }
  }

  // correct for angle overflow
  void OdomEstimation::angleOverflowCorrect(double& a, double ref)
  {
    while ((a-ref) >  M_PI) a -= 2*M_PI;
    while ((a-ref) < -M_PI) a += 2*M_PI;
  }

  // decompose Transform into x,y,z,Rx,Ry,Rz
  void OdomEstimation::decomposeTransform(const geometry_msgs::msg::TransformStamped& trans, 
					   double& x, double& y, double&z, double&Rx, double& Ry, double& Rz){
    x = trans.transform.translation.x;   
    y = trans.transform.translation.y; 
    z = trans.transform.translation.z; 
    tf2::Quaternion orientation;
    tf2::fromMsg(trans.transform.rotation, orientation);
    tf2::Matrix3x3(orientation).getEulerYPR(Rz, Ry, Rx);
  }

  // decompose Transform into x,y,z,Rx,Ry,Rz
  void OdomEstimation::decomposeTransform(const Transform& trans, 
					   double& x, double& y, double&z, double&Rx, double& Ry, double& Rz){
    x = trans.getOrigin().x();   
    y = trans.getOrigin().y(); 
    z = trans.getOrigin().z(); 
    trans.getBasis().getEulerYPR(Rz, Ry, Rx);
  }

  void OdomEstimation::setOutputFrame(const std::string& output_frame){
	output_frame_ = output_frame;
  }

  void OdomEstimation::setBaseFootprintFrame(const std::string& base_frame){
	base_footprint_frame_ = base_frame;
  }

} // namespace
