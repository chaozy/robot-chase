#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import time
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
from scipy.optimize import linear_sum_assignment


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

def get_dist(red, blue):
    return np.linalg.norm(red.pose[:2] - blue.pose[:2])

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
        
        self.num_of_reds = int(self.get_parameter('num_of_robot').value)
        self.num_of_blues = int(self.get_parameter('num_of_robot').value)
        
        self._publishers = [None for _ in range(self.num_of_reds)]
        self._subscribers_blue = [None for _ in range(self.num_of_blues)]
        self._subscribers_red = [None for _ in range(self.num_of_reds)]
        self._reds = [Robot(i) for i in range(self.num_of_reds)]
        self._blues = [Robot(i) for i in range(self.num_of_blues)]
        
        self._catched_set = set()
        

        for idx in range(self.num_of_reds):
            red_idx = idx + self.num_of_blues
            
            pname = "/red/robot" + str(red_idx) + "/cmd_vel"
            self._publishers[idx] = self.create_publisher(Twist, pname, 1)
            self._logger.info(pname)
            
            sname = "/red/robot" + str(red_idx) + "/odometry"
            self._subscribers_red[idx] = self.create_subscription(Odometry, sname, self._reds[idx].update_pose, 1)
            
            # the qos_profile is set to 1 beacause we dont need queueing message, they are outdated
            sname = "/blue/robot" + str(idx) + "/odometry"
            self._subscribers_blue[idx] = self.create_subscription(Odometry, sname, self._blues[idx].update_pose, 1)
            
        time.sleep(10)  
        self.timer = self.create_timer(0.1,  self.robot_chase_timer)
            
    
    def robot_chase_timer(self):
        red_idxes, blue_idxes = self.work_allocation()

        for idx in range(self.num_of_reds):
            red_idx, blue_idx = red_idxes[idx], blue_idxes[idx]
            
            # self._logger.info(str(red_idx + self.num_of_reds) + " chasing " + str(blue_idx))
            velocity = get_relative_pose(self._reds[red_idx].pose, self._blues[blue_idx].pose)
            u, w = linearized_feedback(np.array([0, 0, 0]), velocity=velocity[:2])
    
            vel_msg = Twist()
            vel_msg.linear.x = u
            vel_msg.angular.z = w
            self._publishers[idx].publish(vel_msg)
        return
    
    def work_allocation(self):
        # Compute the cost matrix which contains distance between each pair of robots in the two teams
        cost_m = np.zeros((self.num_of_reds, self.num_of_blues))
        for i in range(self.num_of_reds):
            for j in range(self.num_of_blues):
                if j in self._catched_set:
                    continue
                
                distance = get_dist(self._reds[i], self._blues[j])
                if distance < 0.5:
                    self.robot_catched(j)
                cost_m[i][j] = cost_m[j][i] = distance
        
        # Huangarian Bipartie allocation
        red_idxes, blue_idxes = linear_sum_assignment(cost_matrix=cost_m)
        
        # Find the red team robots who are assigned to the dummy(catched) blue robots, assigh them to their cloest blue robots instead
        for idx in range(self.num_of_reds):
            if blue_idxes[idx] in self._catched_set:
                red_idx = red_idxes[idx]
                non_zeros = cost_m[red_idx][np.nonzero(cost_m[red_idx])] # non-zeros distances of this red robot
                i = np.where(cost_m[red_idx]==np.min(non_zeros))[0][0] # cloest uncatched blue robot
                print ("DWADddddddddddddddddd: ", i)
                blue_idxes[idx] = i
        return red_idxes, blue_idxes
    
    def robot_catched(self, idx):
        self._catched_set.add(idx)
        # When all blue are catched, stop the simulation
        if len(self._catched_set) == self.num_of_blues:
            self.finish()
        self._logger.info(str(idx) + " is catched")
        # the catched robot remains steay
        # vel_msg = Twist()
        # vel_msg.linear.x = 0.
        # vel_msg.angular.z = 0.
        # self._publishers[idx].publish(vel_msg)
        
    def finish(self):
        self._logger.info("ALL BLUE-TEAM ROBOTS ARE CATCHED")
        rclpy.shutdown()


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
