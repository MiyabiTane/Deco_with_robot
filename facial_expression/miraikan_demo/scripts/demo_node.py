#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, Int32MultiArray  # [motion_mode, time_delay]

class MiraikanDemo(object):
    def __init__(self):
        self.mt_pub_msg = Int32()
        self.eb_pub_msg = Int32()

        self.pub_motion_talk = rospy.Publisher("motion_talk/input", Int32, queue_size=1)
        self.pub_eyebrows = rospy.Publisher("eyebrows/input", Int32, queue_size=1)

        rospy.Subscriber("~input", Int32MultiArray, self.demo_cb)
    
    def demo_cb(self, msg):
        # ToDo: 動きと眉毛を組み合わせる
        motion_mode = msg.data[0]
        if len(msg.data) > 1:
            time_delay = msg.data[1]
        else:
            time_delay = 3
        if motion_mode == 0:
            self.mt_pub_msg.data = 0
            self.eb_pub_msg.data = 1
            self.pub_motion_talk.publish(self.mt_pub_msg)
            # 眉毛を送るタイミングをセリフに合わせる
            rospy.sleep(time_delay)
            self.pub_eyebrows.publish(self.eb_pub_msg)
        elif motion_mode == 1:
            self.mt_pub_msg.data = 1
            self.eb_pub_msg.data = 2
            self.pub_motion_talk.publish(self.mt_pub_msg)
            rospy.sleep(time_delay)
            self.pub_eyebrows.publish(self.eb_pub_msg)
        else:
            print("Error out of range")

if __name__ == '__main__':
    rospy.init_node("miraikan_demo")
    miraikan_demmo = MiraikanDemo()
    rospy.spin()
