#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from miraikan_demo.srv import Mode
from std_msgs.msg import Int32

class MiraikanDemo(object):
    def __init__(self):
        self.mt_pub_msg = Int32()
        self.eb_pub_msg = Int32()

        self.pub_motion_talk = rospy.Publisher("motion_talk/input", Int32, queue_size=1)
        self.pub_eyebrows = rospy.Publisher("eyebrows/input", Int32, queue_size=1)
    
    def demo_srv_cb(self, req):
        # ToDo: 動きと眉毛を組み合わせる
        motion_mode = req.mode
        time_delay = req.time_delay
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
        return True


if __name__ == '__main__':
    rospy.init_node("miraikan_demo")
    miraikan_demo = MiraikanDemo()
    s = rospy.Service('demo_mode', Mode, miraikan_demo.demo_srv_cb)
    print("Ready to start demo node")
    rospy.spin()
