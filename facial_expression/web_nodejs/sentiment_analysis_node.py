#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import time
import subprocess
import argparse

from std_msgs.msg import String
from std_msgs.msg import Float64, Bool

class SentimentAnalysis:
    def __init__(self, credentials_path):
        self.score = 0
        self.credentials_path = credentials_path

        self.pub = rospy.Publisher("/degree", Float64, queue_size=1)
        rospy.Subscriber("/robotsound_text", String, self.sound_cb)

    def sound_cb(self, msg):
        subprocess.call(["pipenv", "run", "python", "sentiment_analysis.py",
                         "--credentials-path", self.credentials_path, "--voice-text", msg.data])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--credentials-path")
    args = parser.parse_args()
    rospy.init_node("analysis")
    sentiment_analysis = SentimentAnalysis(args.credentials_path)
    rospy.spin()