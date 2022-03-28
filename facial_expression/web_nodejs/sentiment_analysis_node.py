#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import time
import subprocess

from ros_google_cloud_language.msg import AnalyzeTextAction, AnalyzeTextGoal
from sound_play.msg import SoundRequest
from std_msgs.msg import Float64, Bool

class SentimentAnalysis:
    def __init__(self):
        self.score = 0

        self.pub = rospy.Publisher("/degree", Float64, queue_size=1)

        rospy.Subscriber("/robotsound_jp", SoundRequest, self.sound_cb)
        self.actionlib_client = actionlib.SimpleActionClient("/analyze_text/text", AnalyzeTextAction)
        print("Waiting sever ...")
        self.actionlib_client.wait_for_server()


    def sound_cb(self, msg):
        goal_msg = AnalyzeTextGoal(text=msg.arg)
        self.actionlib_client.send_goal(goal_msg)

        print("Waiting goal ...")
        self.actionlib_client.wait_for_result()
        analysis_result = self.actionlib_client.get_result()

        print("=== sentiment ===")
        self.score = analysis_result.sentiment_score
        print("score: ", self.score)
        print("magnitude", analysis_result.sentiment_magnitude)

        # [ToDo]self.pub.publishだと2回目以降が機能しない。原因解明
        # pub_msg = Float64()
        # pub_msg.data = self.score * 20
        # self.pub.publish(pub_msg)

        # publish
        subprocess.call(["rostopic", "pub", "-1", "/degree", "std_msgs/Float64", "data: " + str(self.score * 20)])


if __name__ == '__main__':
    rospy.init_node("analysis")
    sentiment_analysis = SentimentAnalysis()
    rospy.spin()