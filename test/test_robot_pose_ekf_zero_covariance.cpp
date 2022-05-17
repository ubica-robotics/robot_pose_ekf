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
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace rclcpp;
using namespace std::placeholders;


class TestEKF : public testing::Test
{
public:
  Node::SharedPtr node_;
  Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_sub_;    
  double ekf_counter_;


  void EKFCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr ekf)
  {
    // count number of callbacks
    ekf_counter_++;
  }


protected:
  /// constructor
  TestEKF()
  {
    node_ = std::make_shared<Node>("testEKF");
    ekf_counter_ = 0;

  }


  /// Destructor
  ~TestEKF()
  {
    shutdown();
    std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  }
};




TEST_F(TestEKF, test)
{
  RCLCPP_INFO(node_->get_logger(), "Subscribing to robot_pose_ekf/odom_combined");
  ekf_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/robot_pose_ekf/odom_combined", 10, std::bind(&TestEKF::EKFCallback, this, _1));

  // wait for 20 seconds
  RCLCPP_INFO(node_->get_logger(), "Waiting for 20 seconds while bag is playing");
  Rate d(std::chrono::seconds(20));
  d.sleep();
  RCLCPP_INFO(node_->get_logger(), "End time reached");

  EXPECT_EQ(ekf_counter_, 0);

  SUCCEED();
}




int main(int argc, char** argv)
{
  init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  //g_argc = argc;
  //g_argv = argv;

  //init(g_argc, g_argv, "testEKF");

  //boost::thread spinner(boost::bind(&ros::spin));

  int res = RUN_ALL_TESTS();
  //spinner.interrupt();
  //spinner.join();

  return res;
}
