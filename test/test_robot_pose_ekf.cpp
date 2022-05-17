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

#include <string>
#include <chrono>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


using namespace rclcpp;
using namespace std::placeholders;


static const double time_end     = 1264198883.0;
static const double ekf_duration = 62.0;
static const double EPS_trans_x    = 0.02;
static const double EPS_trans_y    = 0.04;
static const double EPS_trans_z    = 0.00001;
static const double EPS_rot_x      = 0.005;
static const double EPS_rot_y      = 0.005;
static const double EPS_rot_z      = 0.005;
static const double EPS_rot_w      = 0.005;



class TestEKF : public testing::Test
{
public:
  Node::SharedPtr node_;
  Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_sub_;    
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr ekf_begin_, ekf_end_;
  nav_msgs::msg::Odometry::SharedPtr odom_end_;
  double ekf_counter_, odom_counter_;
  Time ekf_time_begin_, odom_time_begin_;

  //void OdomCallback(const OdomConstPtr& odom)
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    // get initial time
    if (odom_counter_ == 0)
      odom_time_begin_ = odom->header.stamp;

    odom_end_ = odom;

    // count number of callbacks
    odom_counter_++;
  }


  //void EKFCallback(const EkfConstPtr& ekf)
  void EKFCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr ekf)
  {
    // get initial time
    if (ekf_counter_ == 0){
      ekf_time_begin_ = ekf->header.stamp;
      ekf_begin_ = ekf;
    }
    if (Time(ekf->header.stamp).seconds() < time_end)
      ekf_end_ = ekf;

    // count number of callbacks
    ekf_counter_++;
  }


protected:

  /// constructor
  TestEKF()
  {
    node_ = std::make_shared<Node>("testEKF");
    ekf_counter_ = 0;
    odom_counter_ = 0;

  }


  /// Destructor
  ~TestEKF()
  {
  }

  void SetUp()
  {
    RCLCPP_INFO(node_->get_logger(), "Subscribing to robot_pose_ekf/odom_combined");
    ekf_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/robot_pose_ekf/odom_combined", 10, std::bind(&TestEKF::EKFCallback, this, _1));

    RCLCPP_INFO(node_->get_logger(), "Subscribing to base_odometry/odom");
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("base_odometry/odom", 10, std::bind(&TestEKF::OdomCallback, this, _1));

  }

  void TearDown()
  {
    //odom_sub_.shutdown(); TODO
    //ekf_sub_.shutdown(); TODO
    shutdown();
    std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  }
  
};




TEST_F(TestEKF, test)
{
  Rate d1(std::chrono::milliseconds(10));
  // wait while bag is played back
  RCLCPP_INFO(node_->get_logger(), "Waiting for bag to start playing");
  while (odom_counter_ == 0)
    d1.sleep();
  RCLCPP_INFO(node_->get_logger(), "Detected that bag is playing");

  RCLCPP_INFO(node_->get_logger(), "Waiting untile end time is reached");
  while( Time(odom_end_->header.stamp).seconds() < time_end){
    d1.sleep();
  }
  RCLCPP_INFO(node_->get_logger(), "End time reached");
  // give filter some time to catch up
  Rate d2(std::chrono::seconds(2));
  d2.sleep();

  // check if callback was called enough times
  RCLCPP_INFO(node_->get_logger(), "Number of ekf callbacks: %f", ekf_counter_);
  EXPECT_GT(ekf_counter_, 500);

  // check if time interval is correct
  RCLCPP_INFO(node_->get_logger(), "Ekf duration: %f", ekf_duration);
  EXPECT_LT(Duration(Time(ekf_end_->header.stamp) - ekf_time_begin_).seconds(), ekf_duration * 1.25);
  EXPECT_GT(Duration(Time(ekf_end_->header.stamp) - ekf_time_begin_).seconds(), ekf_duration * 0.85);

  // check if ekf time is same as odom time
  EXPECT_NEAR(ekf_time_begin_.seconds(),  odom_time_begin_.seconds(), 1.0);
  EXPECT_NEAR(Time(ekf_end_->header.stamp).seconds(), time_end, 1.0);

  // check filter result
  RCLCPP_INFO(node_->get_logger(), "%f %f %f %f %f %f %f -- %f",ekf_end_->pose.pose.position.x, ekf_end_->pose.pose.position.y, ekf_end_->pose.pose.position.z,
            ekf_end_->pose.pose.orientation.x, ekf_end_->pose.pose.orientation.y, ekf_end_->pose.pose.orientation.z, ekf_end_->pose.pose.orientation.w,
            Time(ekf_end_->header.stamp).seconds());
  EXPECT_NEAR(ekf_end_->pose.pose.position.x, -0.0586126, EPS_trans_x);
  EXPECT_NEAR(ekf_end_->pose.pose.position.y, 0.0124321, EPS_trans_y);
  EXPECT_NEAR(ekf_end_->pose.pose.position.z, 0.0, EPS_trans_z);
  EXPECT_NEAR(ekf_end_->pose.pose.orientation.x, 0.00419421,  EPS_rot_x);
  EXPECT_NEAR(ekf_end_->pose.pose.orientation.y,  0.00810739, EPS_rot_y);
  EXPECT_NEAR(ekf_end_->pose.pose.orientation.z, -0.0440686,  EPS_rot_z);
  EXPECT_NEAR(ekf_end_->pose.pose.orientation.w, 0.998987,  EPS_rot_w);

  SUCCEED();
}




int main(int argc, char** argv)
{
  init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  //g_argc = argc;
  //g_argv = argv;

  //init(g_argc, g_argv, "testEKF");
  //auto node = Node::make_shared("testEKF");

  //boost::thread spinner(boost::bind(&ros::spin));
  //std::thread spinner(&spin, node);
  //spinner.detach();

  int res = RUN_ALL_TESTS();
  //spinner.interrupt();
  //spinner.join();
  //while (rclcpp::ok()){
  //    rclcpp::spin(node);
  //}

  return res;
}
