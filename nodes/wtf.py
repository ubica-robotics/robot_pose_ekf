#!/usr/bin/env python

import sys
import rclpy
from robot_pose_ekf.srv import GetStatus

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('spawner')
    print ('looking for node robot_pose_ekf...')

    get_status = node.create_client(GetStatus, 'robot_pose_ekf/get_status')
    while not get_status.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
        continue
    future = get_status.call_async(GetStatus.Request())
    while rclpy.ok():
        rclpy.spin_once(node)
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                node.get_logger().info(
                    'Result of get_status service: %s' %
                    (response.status))
            break

    node.destroy_node()
    rclpy.shutdown()

