#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import subprocess
from std_msgs.msg import Int32

class TopicToEyebrows(object):
    def __init__(self):
        rospy.Subscriber("/eyebrows/input_type", Int32, self.eyebrows_cb)
    
    def eyebrows_cb(self, msg):
        degrees = [0, 20, 30, 70, 60, 20, 30, 30, 70, 120, 70, 140, 20]
        subprocess.call(["curl", "-X", "POST", "--data-urlencode", "mode=" + str(msg.data),
                         "--data-urlencode", "degree=" + str(degrees[msg.data]), "http://localhost:3000/api/info"])

if __name__ == '__main__':
    rospy.init_node("topic_to_eyebrows")
    topic_to_eyebrows = TopicToEyebrows()
    rospy.spin()
