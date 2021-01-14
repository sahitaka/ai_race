#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import math
import sys
import time
import random
import tf

from std_msgs.msg import Float64
from cob_srvs.srv import SetInt, SetIntRequest
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from geometry_msgs.msg import Twist, Pose

class StackDetector:

    def __init__(self):
        self.get_rosparam()
        # rospy.Subscriber("/rear_left_wheel_velocity_controller/command", Float64, self.get_left_command)
        # rospy.Subscriber("/rear_right_wheel_velocity_controller/command", Float64, self.get_right_command)
        rospy.Subscriber("/cmd_vel", Twist, self.get_target_command)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.get_speed)
        self.respown_srv = rospy.ServiceProxy("/jugemu_new/teleport", SetModelState)
        self.last_ramdom = 0.0
        self.pose = Pose()
        
    def get_rosparam(self):
        self.respown_point = rospy.get_param("/respown_point")

    def get_target_command(self, cmd):
        self.target_command = cmd.linear.x


    def get_speed(self, models):
        try:
            index = models.name.index("wheel_robot")
            self.pose = models.pose[index]
        except ValueError:
            #print ('can not get model.name.index, skip !!')
            pass
        


    def compare_command_value(self):
        if rospy.Time.now().to_sec() - self.last_ramdom > 5:
            res_pown_num = len(self.respown_point)
            n = random.randint(0,res_pown_num-1)
            print self.respown_point[n]
            self.last_ramdom = rospy.Time.now().to_sec()
            req = SetModelStateRequest()
            req.model_state.pose.position.x = self.respown_point[n][0] + random.uniform(-0.3, 0.3)
            req.model_state.pose.position.y = self.respown_point[n][1] + random.uniform(-0.3, 0.3)
            yaw = (self.respown_point[n][2] + random.randint(-10, 10))*math.pi/180
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            req.model_state.pose.orientation.x = q[0]
            req.model_state.pose.orientation.y = q[1]
            req.model_state.pose.orientation.z = q[2]
            req.model_state.pose.orientation.w = q[3]
            self.respown_srv.call(req)
        
        

    def start(self):
        rospy.init_node("ramdom_respown")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.compare_command_value()
            rate.sleep()

def main():
    node = StackDetector()
    node.start()

if __name__ == "__main__":
    main()

