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
    
    def pub_topics(self, mt_int, eb_int, time_delay):
        self.mt_pub_msg.data = mt_int
        self.eb_pub_msg.data = eb_int
        self.pub_motion_talk.publish(self.mt_pub_msg)
        rospy.sleep(time_delay)
        self.pub_eyebrows.publish(self.eb_pub_msg)

    def demo_srv_cb(self, req):
        motion_mode = req.mode
        time_delay = req.time_delay
        if motion_mode == 0:
            # 8年が経ったね😳
            self.pub_topics(motion_mode, 7, time_delay)
        elif motion_mode == 1:
            # 振り向いてもらえなくて悲しかったよ😭
            self.pub_topics(motion_mode, 12, time_delay)
        elif motion_mode == 2:
            # 見つけてくれたよね😀
            self.pub_topics(motion_mode, 1, time_delay)
        else:
            print("Error out of range")
        return True


if __name__ == '__main__':
    rospy.init_node("miraikan_demo")
    miraikan_demo = MiraikanDemo()
    s = rospy.Service('demo_mode', Mode, miraikan_demo.demo_srv_cb)
    print("Ready to start demo node")
    rospy.spin()
