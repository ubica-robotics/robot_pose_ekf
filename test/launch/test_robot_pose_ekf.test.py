# -*- coding: utf-8 -*-
import launch
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
import launch_testing
import os
import sys
import unittest
from launch.actions import ExecuteProcess
import launch.event_handlers.on_process_start
from ament_index_python.packages import get_package_share_directory


def generate_test_description():

    pkg_dir = get_package_share_directory('robot_pose_ekf')

    use_sim_time = 'true' #find better way of doing this

    test_robot_pose_ekf = Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration("test_binary_dir"),
                "test_robot_pose_ekf",
            ]
        ),
    )

    # Required for obtaining realtime
    # logging on the screen
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    start_gazebo_server_cmd = ExecuteProcess(
        condition=launch.conditions.IfCondition(use_sim_time),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', 'empty.world']
    )

    # Create a process running the executable
    odom_node_proc = ExecuteProcess(
        cmd=["ros2", "run", "robot_pose_ekf", "robot_pose_ekf", "--ros-args", 
             "-r", "odom:=base_odometry/odom", "-r",  "imu_data:=torso_lift_imu/data", 
             "--params-file", os.path.join(pkg_dir, 'config', "test_robot_pose_ekf_params.yaml")],
        shell=True,
        env=proc_env
    )


    # Create a process running the executable
    rosbag_proc = ExecuteProcess(
        cmd=["ros2", "bag", "play", "--clock", "100", "-d", ".4", os.path.join(pkg_dir, 'data', "ekf_test2_indexed")],
        shell=True,
        env=proc_env
    )

    return launch.LaunchDescription(
        [
                
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            ExecuteProcess(
                cmd=['ros2', 'daemon', 'stop'],
                name='daemon-stop',
                on_exit=[
                    start_gazebo_server_cmd,
                ]
            ),
            ExecuteProcess(
                cmd=['ros2', 'daemon', 'stop'],
                name='daemon-stop',
                on_exit=[
                    odom_node_proc,
                    launch_testing.actions.ReadyToTest()
                ]
            ),
            ExecuteProcess(
                cmd=['ros2', 'daemon', 'stop'],
                name='daemon-stop',
                on_exit=[
                    rosbag_proc
                ]
            ),
            test_robot_pose_ekf
        ]
    ), {
        "test_robot_pose_ekf": test_robot_pose_ekf,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, proc_info, test_robot_pose_ekf):
        proc_info.assertWaitForShutdown(test_robot_pose_ekf, timeout=6000.0)

