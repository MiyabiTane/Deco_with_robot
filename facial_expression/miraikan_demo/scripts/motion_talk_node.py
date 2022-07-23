#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32

import episode_motion
import no_episode_motion

class TopicToMotion(object):
    def __init__(self):
        pepper_ip = rospy.get_param("~pepper_ip", "")
        memories_talk = rospy.get_param("~memories_talk", True)
        print("ip: ", pepper_ip, "mode: ", memories_talk)
        if memories_talk:
            self.talk_class = episode_motion.Talk(pepper_ip)
        else:
            self.talk_class = no_episode_motion.Talk(pepper_ip)

        rospy.Subscriber("~input", Int32, self.motion_cb)
    
    def motion_cb(self, msg):
        if msg.data == 0:
            self.talk_class.episode_11()
        elif msg.data == 1:
            self.talk_class.episode_12()
        elif msg.data == 2:
            self.talk_class.episode_13()
        elif msg.data == 3:
            self.talk_class.episode_21()
        elif msg.data == 4:
            self.talk_class.episode_22()
        elif msg.data == 5:
            self.talk_class.episode_23()
        elif msg.data == 6:
            self.talk_class.episode_31()
        elif msg.data == 7:
            self.talk_class.episode_32_1()
        elif msg.data == 8:
            self.talk_class.episode_32_2()
        elif msg.data == 9:
            self.talk_class.episode_33_1()
        elif msg.data == 10:
            self.talk_class.episode_33_2()
        elif msg.data == 11:
            self.talk_class.episode_41()
        elif msg.data == 12:
            self.talk_class.episode_42_1()
        elif msg.data == 13:
            self.talk_class.episode_42_2()
        elif msg.data == 14:
            self.talk_class.episode_43()
        elif msg.data == 15:
            self.talk_class.episode_51()
        elif msg.data == 16:
            self.talk_class.episode_52()
        elif msg.data == 17:
            self.talk_class.episode_53()
        elif msg.data == 18:
            self.talk_class.episode_54_1()
        elif msg.data == 19:
            self.talk_class.episode_54_2()
        else:
            print("Error, out of range")

if __name__ == '__main__':
    rospy.init_node("topic_to_motion")
    topic_to_motion = TopicToMotion()
    rospy.spin()
