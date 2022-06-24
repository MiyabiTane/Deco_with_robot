#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_msgs.msg import String
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sound_play.msg import SoundRequestAction, SoundRequestGoal

class TextToSound(object):
    def __init__(self):
        self.volume = 1.0
        self.command = 1
        self.sound = -3
        self.arg2 = 'ja'

        rospy.Subscriber("/text", String, self.speak_cb)

        self.actionlib_client = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        print("waiting server ...")
        self.actionlib_client.wait_for_server()
        print("Server Called")
    
    def speak_cb(self, msg):
        speak_msg = SoundRequestGoal()
        speak_msg.sound_request.volume = self.volume
        speak_msg.sound_request.command = self.command
        speak_msg.sound_request.sound = self.sound
        speak_msg.sound_request.arg = msg.data
        speak_msg.sound_request.arg2 = self.arg2
        self.actionlib_client.send_goal(speak_msg)

if __name__ == '__main__':
    print("main run")
    rospy.init_node("text_to_sound")
    text_to_sound = TextToSound()
    rospy.spin()
