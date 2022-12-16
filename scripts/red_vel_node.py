#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

# For pose information.
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry

from launch import LaunchDescription
from launch_ros.actions import Node as nd

import numpy as np
from scipy.spatial.transform import Rotation as R


X = 0
Y = 1
YAW = 2

def get_relative_pose(robot, target):
    relative = robot.copy()
    t1 = -robot[YAW]
    delta = target[:2] - robot[:2]
    relative[X] = np.cos(t1) * delta[X] - np.sin(t1) * delta[Y]
    relative[Y] = np.sin(t1) * delta[X] + np.cos(t1) * delta[Y]
    
    return relative

def linearized_feedback(pose, velocity):
    epsilon = .2
    x, y, theta = pose
    vp_x, vp_y = velocity
    u = float(vp_x * np.cos(theta) + vp_y * np.sin(theta))
    w = float(epsilon ** -1 * (-vp_x * np.sin(theta) + vp_y * np.cos(theta)))
    return u, w

class Robot():
    def __init__(self, id):
        self.id = id
        self._pose = np.array([np.nan, np.nan, np.nan])

    def update_pose(self, msg):
        self._pose[X], self._pose[Y] = msg.pose.pose.position.x, msg.pose.pose.position.y
        _, _, self._pose[YAW] = R.from_quat([
                                            msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z,
                                            msg.pose.pose.orientation.w]).as_euler('XYZ')
    
    @property
    def pose(self):
        return self._pose


class RedbotDriver(Node):
    def __init__(self, args):
        super().__init__('Redbot_driver')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('num_of_robot', 1)
            ])
        
        self.num_of_robots = int(self.get_parameter('num_of_robot').value)
        
        self._publishers = []
        self._subscribers_blue = []
        self._subscribers_red = []
        self._reds = [Robot(i) for i in range(self.num_of_robots)]
        self._blues = [Robot(i) for i in range(self.num_of_robots)]
        

        for idx in range(self.num_of_robots):
            red_idx = idx + self.num_of_robots
            
            self._logger.info(str(red_idx))
            
            pname = "/red/robot" + str(red_idx) + "/cmd_vel"
            self._publishers.append(self.create_publisher(Twist, pname, 1))
            
            sname = "/red/robot" + str(red_idx) + "/odometry"
            self._subscribers_red.append(self.create_subscription(Odometry, sname, self._reds[idx].update_pose, 1)) 
            
            # the qos_profile is set to 1 beacause we dont need queueing message, they are outdated
            sname = "/blue/robot" + str(idx) + "/odometry"
            self._subscribers_blue.append(self.create_subscription(Odometry, sname, self._blues[idx].update_pose, 1))
            
            
            
        self.timer = self.create_timer(1,  self.robot_chase_timer)
            
    
    def robot_chase_timer(self):

        for idx in range(self.num_of_robots):
            velocity = get_relative_pose(self._reds[idx].pose, self._blues[idx].pose)
            u, w = linearized_feedback(np.array([0, 0, 0]), velocity=velocity[:2])
    
            vel_msg = Twist()
            vel_msg.linear.x = u
            vel_msg.angular.z = w
            self._publishers[idx].publish(vel_msg)
        return



def run(args):
    rclpy.init()

    multirobot_driver_node = RedbotDriver(args)

    rclpy.spin(multirobot_driver_node)

    multirobot_driver_node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Runs red-robot navigation')
    args, unknown = parser.parse_known_args()
    run(args)


if __name__ == '__main__':
    main()
