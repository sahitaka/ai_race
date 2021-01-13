#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from common import CmdManager
from std_msgs.msg import Bool


class AutoRun:

    def __init__(self):
        rospy.Subscriber("/front_camera/image_raw", Image, self.get_image)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.bridge = CvBridge()
        self.cv_image = np.ndarray((1,1))
        self.horizontal_fov = 2.0
        self.cmd_manager = CmdManager(1.0, -1.0, 9)
        self.is_dummy = False

    def get_image(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def estimate_target_point(self):
        yellow_lower = np.array([0, 200, 200])
        yellow_upper = np.array([50, 255, 255])
        img_mask = cv2.inRange(self.cv_image, yellow_lower, yellow_upper)
        x, y = np.where(img_mask == 255)
        if np.size(x) == 0:
            x = 0
            y = [0, 0]
        target_x = np.min(x)
        target_y = y[np.argmin(x)]
        return target_x, target_y


    def estimate_cmd(self):
        target_x, target_y = self.estimate_target_point()
        angle = self.horizontal_fov/2 + (2.0/self.cv_image.shape[1])*(-target_y)
        cmd_vel = Twist()
        cmd_vel.linear.x = 1.6
        cmd_vel.angular.z = self.cmd_manager.resample_cmd(angle*2.0)
        # cmd_vel.angular.z = angle*2.0
        print angle, cmd_vel.angular.z
        self.cmd_pub.publish(cmd_vel)
        cv2.circle(self.cv_image, (target_y, target_x), 5, [0, 255, 0])
        cv2.imshow("result", self.cv_image)
        cv2.waitKey(1)




    def start(self):
        rospy.init_node("estimate_cmd")
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            self.estimate_cmd()
            rate.sleep()

def main():
    node = AutoRun()
    node.start()

if __name__ == "__main__":
    main()
