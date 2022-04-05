#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess

from std_msgs.msg import Float64, Bool

class FacialExpressionNode:
    def __init__(self):
        self.degree = 0
        rospy.Subscriber('/facial_expression/degree', Float64, self.face_exp_cb)
        subprocess.call(["curl", "-X", "POST", "--data-urlencode", "degree=0", "http://localhost:3000/api/info"])

    def face_exp_cb(self, msg):
        self.degree = msg.data
        print("== degree: ==", self.degree)
        subprocess.call(["curl", "-X", "POST", "--data-urlencode", "degree=" + str(self.degree), "http://localhost:3000/api/info"])


if __name__ == '__main__':
    rospy.init_node("facial_exp")
    facial_exp = FacialExpressionNode()
    rospy.spin()
