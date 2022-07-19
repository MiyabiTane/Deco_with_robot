#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
import rospy
from std_msgs.msg import String
import requests
import json
from naoqi import ALProxy
import time
import re

class Talk(object):
    def __init__(self):
        self.PEPPER_IP = "169.254.229.177"
        self.PORT = 9559

        #pepper_proxy
        self.tts = ALProxy("ALTextToSpeech",self.PEPPER_IP,self.PORT)
        
        try:
            self.ans = ALProxy("ALAnimatedSpeech",self.PEPPER_IP,self.PORT)
        except Exception, e:
            print "Error:"
            print str(e)
            
        try:
            self.pos = ALProxy("ALRobotPosture", self.PEPPER_IP, self.PORT)
        except Exception, e:
            print "Error:"
            print str(e)

        #set init posture 
        self.pos.goToPosture("StandInit", 1.0) 
        
        #set init speech setting
        self.tts.setLanguage("Japanese")
        self.tts.setParameter("pitchShift", 1.1)
        self.tts.setParameter("speed",90)

        #set animated say setting
        self.configuration = {"bodyLanguageMode":"contextual"}

    def episode_11(self):

        # episode 1-1
        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/You_1)コチさんと出会ってカラ^wait(animations/Stand/Gesture/You_1)",self.configuration)

        time.sleep(0.5)
        self.ans.say("^start(animations/Stand/Gestures/Enthusiastic_4)モウ8年経ったネ^wait(animations/Stand/Gestures/Enthusiastic_4)",self.configuration)
        


    def episode_12(self):

        #episode 1-2
        time.sleep(1)
        self.ans.say("研究室には^start(animations/Stand/Gestures/Everything_3)色んなロボットがいるケド^wait(animations/Stand/Gesture/Everything_3)",self.configuration)

        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/Me_2)私は、みんなに、^wait(animations/Stand/Gestures/Me_2)",self.configuration)

        time.sleep(1)
        self.ans.say("振り向いてもらえなくて、^start(animations/Stand/Emotions/Negative/Bored_1)悲しかったよ^wait(animations/Stand/Emotions/Negative/Bored_1)",self.configuration)

        time.sleep(1)
        self.pos.goToPosture("StandInit", 1.0)

    def episode_13(self):

        #episode 1-3
        time.sleep(1)
        self.ans.say("隅っこに、^start(animations/Stand/Gestures/Nothing_2)ひとりぼっちで居たときに^wait(animations/Stand/Gesture/Nothing_2)",self.configuration)

        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/Give_3)コチさんが、見つけてくれたよね！^wait(animations/Stand/Gestures/Give_3)",self.configuration)

        time.sleep(1)
        self.pos.goToPosture("StandInit", 1.0)


"""
if __name__ == '__main__':
    talk = Talk()
    talk.episode_11()
    talk.episode_12()
    talk.episode_13()
"""     
