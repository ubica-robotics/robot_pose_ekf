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

#ifndef __ODOM_ESTIMATION_NODE__
#define __ODOM_ESTIMATION_NODE__

// ros stuff
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "odom_estimation.hpp"
#include "robot_pose_ekf/srv/get_status.hpp"

// messages
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <mutex>

// log files
#include <fstream>

namespace estimation
{

/** \mainpage
 *  \htmlinclude manifest.html
 * 
 * <b>Package Summary</b>
 * This package provides two main classes: 
 *  1) OdomEstimation performs all sensor fusion operations, and 
 *  2) OdomEstimationNode provides a ROS wrapper around OdomEstimation
*/

class OdomEstimationNode
{
public:
  /// constructor
  OdomEstimationNode(const rclcpp::Node::SharedPtr node);

  /// destructor
  virtual ~OdomEstimationNode();

private:
  /// the mail filter loop that will be called periodically
  void spin();

  /// callback function for odo data
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  /// callback function for imu data
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu);

  /// callback function for vo data
  void voCallback(const nav_msgs::msg::Odometry::SharedPtr vo);

  /// callback function for vo data
  void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr gps);


  /// get the status of the filter
  bool getStatus(const robot_pose_ekf::srv::GetStatus::Request::SharedPtr req, robot_pose_ekf::srv::GetStatus::Response::SharedPtr resp);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Logger logger_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, vo_sub_, gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<robot_pose_ekf::srv::GetStatus>::SharedPtr state_srv_;
  // ekf filter
  OdomEstimation my_filter_;

  // estimated robot pose message to send
  geometry_msgs::msg::PoseWithCovarianceStamped  output_; 

  // robot state
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  // vectors
  tf2::Transform odom_meas_, imu_meas_, vo_meas_, gps_meas_;
  tf2::Transform base_vo_init_;
  tf2::Transform base_gps_init_;
  geometry_msgs::msg::TransformStamped camera_base_;
  rclcpp::Time odom_time_, imu_time_, vo_time_, gps_time_;
  rclcpp::Time odom_stamp_, imu_stamp_, vo_stamp_, gps_stamp_, filter_stamp_;
  rclcpp::Time odom_init_stamp_, imu_init_stamp_, vo_init_stamp_, gps_init_stamp_;
  bool odom_active_, imu_active_, vo_active_, gps_active_;
  bool odom_used_, imu_used_, vo_used_, gps_used_;
  bool odom_initializing_, imu_initializing_, vo_initializing_, gps_initializing_;
  double timeout_;
  MatrixWrapper::SymmetricMatrix odom_covariance_, imu_covariance_, vo_covariance_, gps_covariance_;
  bool debug_, self_diagnose_;
  std::string output_frame_, base_footprint_frame_, frame_prefix_;

  // log files for debugging
  std::ofstream odom_file_, imu_file_, vo_file_, gps_file_, corr_file_;

  // counters
  unsigned int odom_callback_counter_, imu_callback_counter_, vo_callback_counter_,gps_callback_counter_, ekf_sent_counter_;

}; // class

} // namespace

#endif
