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

    test_robot_pose_ekf = Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration("test_binary_dir"),
                "test_robot_pose_ekf",
            ]
        ),
        output="screen",
    )

    # Required for obtaining realtime
    # logging on the screen
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    # Create a process running the executable
    odom_node_proc = ExecuteProcess(
        cmd=["ros2", "run", "robot_pose_ekf", "robot_pose_ekf", "--clock", "100", "-d", ".4", "--ros-args", 
             "-r", "odom:=base_odometry/odom", "-r",  "imu_data:=torso_lift_imu/data", 
             "--params-file", os.path.join(pkg_dir, 'data', "test_robot_pose_ekf_params.yaml")],
        shell=True,
        env=proc_env,
        output='screen'
    )


    # Create a process running the executable
    rosbag_proc = ExecuteProcess(
        cmd=["ros2", "bag", "play", os.path.join(pkg_dir, 'data', "ekf_test2_indexed")],
        shell=True,
        env=proc_env,
        output='screen'
    )

    return launch.LaunchDescription(
        [
            SetParameter(name='use_sim_time', value=True),
                
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            test_robot_pose_ekf,
            # TimerAction(period=2.0, actions=[basic_test]),
            ExecuteProcess(
                cmd=['ros2', 'daemon', 'stop'],
                name='daemon-stop',
                on_exit=[
                    # This argument can be passed into the ros2_integration_test, and can be discovered by running
                    # launch_test --show-args
                    odom_node_proc,
                    # In tests where all of the procs under tests terminate themselves, it's necessary
                    # to add a dummy process not under ros2_integration_test to keep the launch alive. launch_test
                    # provides a simple launch action that does this:
                    launch_testing.util.KeepAliveProc(),
                    launch_testing.actions.ReadyToTest()
                ]
            ),
            ExecuteProcess(
                cmd=['ros2', 'daemon', 'stop'],
                name='daemon-stop',
                on_exit=[
                    # This argument can be passed into the ros2_integration_test, and can be discovered by running
                    # launch_test --show-args
                    rosbag_proc,
                    # In tests where all of the procs under tests terminate themselves, it's necessary
                    # to add a dummy process not under ros2_integration_test to keep the launch alive. launch_test
                    # provides a simple launch action that does this:
                    launch_testing.util.KeepAliveProc(),
                    launch_testing.actions.ReadyToTest()
                ]
            )
        ]
    ), {
        "test_robot_pose_ekf": test_robot_pose_ekf,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, proc_info, test_robot_pose_ekf):
        proc_info.assertWaitForShutdown(test_robot_pose_ekf, timeout=6000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes
    def test_gtest_pass(self, proc_info, test_robot_pose_ekf):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=test_robot_pose_ekf
        )
