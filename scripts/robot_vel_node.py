#!/usr/bin/env python3

import argparse
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from launch import LaunchDescription
from launch_ros.actions import Node as nd


class Robot_vel(Node):
    def __init__(self, args):
        super().__init__('robot_vel_node')

        self._publisher = self.create_publisher(Twist, '/robot4/cmd_vel', 5)
        vel_msg = Twist()
        vel_msg.linear.x = 0.1
        self._publisher.publish(vel_msg)


        self.timer = self.create_timer(2, self.publish_vel)
        # print ("Start to to publish v elocity")
        self._logger.info("pubish")


    def publish_vel(self):
        vel_msg = Twist()
        vel_msg.angular.z = .5
        vel_msg.linear.x = 0.0
        self._publisher.publish(vel_msg)

        vel_msg.angular.z = 0.
        vel_msg.linear.x = .0
        self._publisher.publish(vel_msg)
    


def generate_launch_description():
    return LaunchDescription([
        nd(
            package='robot_chase',
            executable='robot_vel_node.py',
            name='robot_vel_node',
            output='screen',
            namespace='robot3'),
    ])


def main():
    parser = argparse.ArgumentParser(description='Pub vel to robot_x')
    args, unknown = parser.parse_known_args()
    rclpy.init()

    robot_vel = Robot_vel(args)

    rclpy.spin(robot_vel)

    robot_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
