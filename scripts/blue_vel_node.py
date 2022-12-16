#!/usr/bin/env python3

import argparse
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from launch import LaunchDescription
from launch_ros.actions import Node as nd

import time
import numpy as np

VELOCITY = 2.

class Robot_vel(Node):
    def __init__(self, args):
        super().__init__('robot_vel_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('num_of_robot', 1)
            ])
        
        num_of_robot = int(self.get_parameter('num_of_robot').value)
        self._publishers = []
        

        
        for idx in range(num_of_robot):
            rname = "/blue/robot" + str(idx) + "/cmd_vel"
            self._logger.info("published to: " + rname)
            self._publishers.append(self.create_publisher(Twist, rname, 2))
            init_vel = Twist()
            init_vel.linear.x = VELOCITY
            self._publishers[idx].publish(init_vel)


        self.timer = self.create_timer(2, self.publish_vel)
        self._logger.info("published")


    def publish_vel(self):
        for publisher in self._publishers:
            vel_msg = Twist()
            # vel_msg.linear.x = 0.0
            vel_msg.angular.z = np.random.rand() * 2
            publisher.publish(vel_msg)
            
        time.sleep(1)    
        for publisher in self._publishers:    
            
            vel_msg.linear.x = VELOCITY
            vel_msg.angular.z = .0
            publisher.publish(vel_msg)

    
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
