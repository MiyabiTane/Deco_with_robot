#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32

from fake_motion_talk import Talk

class TopicToMotion(object):
    def __init__(self):
        self.talk_class = Talk()

        rospy.Subscriber("~input", Int32, self.motion_cb)
    
    def motion_cb(self, msg):
        if msg.data == 0:
            self.talk_class.test_func_0()
        elif msg.data == 1:
            self.talk_class.test_func_1()
        else:
            print("Error, out of range")

if __name__ == '__main__':
    rospy.init_node("topic_to_motion")
    topic_to_motion = TopicToMotion()
    rospy.spin()
