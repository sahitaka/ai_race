#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import tf
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np
import os


class AutoRun:

    def __init__(self):
        rospy.Subscriber("/wheel_robot_tracker", Odometry, self.get_robot_pos)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_vel = Twist()

        home_path = os.getenv('HOME')
        self.path = np.loadtxt(home_path+'/catkin_ws/src/ai_race/ai_race/your_environment/scripts/path.csv', delimiter=',')
        self.pos = Odometry()
        self.velocity = 2.0
        self.look_ahead_dist = 1
        self.look_ahead_n = 10


    def get_robot_pos(self, msg):
        self.pos = msg


    def get_look_ahead(self):
        x = self.pos.pose.pose.position.x
        y = self.pos.pose.pose.position.y

        distance = np.power(np.sum(np.power(self.path - [x, y], 2), 1), 0.5)
        robot_pos_idx = np.argmin(np.abs(distance))
        # if len(distance) - robot_pos_idx < 10:
        #     candidate = np.stack(self.path[robot_pos_idx:-1], self.path[1:10])
        # else:
        #     candidate = np.stack(self.path[robot_pos_idx:robot_pos_idx+20])
        # print candidate
        # distance = np.power(np.sum(np.power(self.path - [x, y], 2), 1), 0.5)
        if len(distance) - robot_pos_idx < self.look_ahead_n+1:
            return len(distance) - robot_pos_idx
        return robot_pos_idx + self.look_ahead_n


    def calc_cmd(self, idx):
        look_ahead_pos = self.path[idx, :]
        
        x = self.pos.pose.pose.position.x
        y = self.pos.pose.pose.position.y
        quaternion = [self.pos.pose.pose.orientation.x,\
                      self.pos.pose.pose.orientation.y,\
                      self.pos.pose.pose.orientation.z, \
                      self.pos.pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion,axes='sxyz')
        
        robot_to_look_ahead_angle = np.arctan2(look_ahead_pos[1]-y, look_ahead_pos[0]-x)
        
        alpha = robot_to_look_ahead_angle - euler[2]

        omg = 2*self.velocity*np.sin(alpha)/(self.look_ahead_n*0.1)
        if omg > 1.5:
            velocity = (self.look_ahead_n*0.1)/(2*np.sin(alpha))
            omg = 1.5
        elif omg < -1.5:
            velocity = (self.look_ahead_n*0.1)/(2*np.sin(alpha))
            omg = -1.5
        else:
            velocity = self.velocity

            

        self.cmd_vel.angular.z = omg
        self.cmd_vel.linear.x = velocity
        self.cmd_pub.publish(self.cmd_vel)
        print x, y
        print look_ahead_pos
        print robot_to_look_ahead_angle, euler[2]
        print self.cmd_vel



    def main(self):
        print 'start'
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            idx = self.get_look_ahead()
            self.calc_cmd(idx)
            rate.sleep()

def main():
    rospy.init_node('auto_run2')
    node = AutoRun()
    node.main()


if __name__ == "__main__":
    main()