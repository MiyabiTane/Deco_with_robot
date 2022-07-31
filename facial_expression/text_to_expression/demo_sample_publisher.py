#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess
from std_msgs.msg import String

DEMO_SENTENCES = ["まいくてすと", "一緒にできて嬉しいな", "お疲れ様", "ぶっちゃけ予想通りだわ", "凄いですね", 
                   "いい話ですね、感動した", "ふざけないで", "照れちゃう", "恐ろしいエピソードだ",
                   "癒やされます", "どうだ、騙されたか〜", "ふーん", "私には難しいです"]

class FacialExpressionSample(object):

    def __init__(self):
        self.sample_words = DEMO_SENTENCES
        self.pub = rospy.Publisher("/text", String, queue_size=1)
        # use when option --with-speak
        self.pub_sound = rospy.Publisher("/text_for_sound", String, queue_size=1)

        rospy.Subscriber("/demo_input", String, self.demo_cb)
    
    def demo_cb(self, msg):
        pub_msg = String()
        pub_msg.data = msg.data
        self.pub.publish(pub_msg)
        rospy.sleep(1)
        self.pub_sound.publish(pub_msg)
    
    def main(self):
        rospy.sleep(5)
        for sample_w in self.sample_words:
            subprocess.call(["rostopic", "pub", "-1", "/demo_input", "std_msgs/String", "data: " + sample_w])
            rospy.sleep(10)

if __name__ == '__main__':
    rospy.init_node("demo_sample")
    facial_expression_sample = FacialExpressionSample()
    facial_expression_sample.main()
