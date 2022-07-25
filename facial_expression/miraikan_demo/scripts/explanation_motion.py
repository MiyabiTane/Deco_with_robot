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
    def __init__(self, pepper_ip):
        self.PEPPER_IP = pepper_ip
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

        try:
            self.mo = ALProxy("ALMotion",self.PEPPER_IP,self.PORT)
        except Exception, e:
            print "Error:"
            print str(e)

        #set init posture                                                                           
        self.set_init_posture()

        
        #set init speech setting
        self.tts.setLanguage("Japanese")
        self.tts.setParameter("pitchShift", 1.1)
        self.tts.setParameter("speed",90)

        #set animated say setting
        self.configuration = {"bodyLanguageMode":"contextual"}
        self.joint_names = "Head"

    def set_init_posture(self):
       # https://developer.softbankrobotics.com/pepper-naoqi-25/naoqi-developer-guide/naoqi-apis/naoqi-motion/almotion/joint-control                                                                    
        # self.pos.goToPosture("StandInit", 1.0)                                                    
        # self.mo.setAngles("Head", [-2.802596928649634e-45, 0.0], 1.0)                             
        #  # commandAngles = self.mo.getAngles("Head", False)                                       
        #  # [-2.802596928649634e-45, -0.20000003278255463]
        # angles = talk.mo.getAngles("Body", False)                                                 
        init_body_angles = [0.0, -2.802596928649634e-45, 1.5596766471862793, 0.14272688329219818, -1.228257656097412, -0.5225345492362976, -0.000497947505209595, 0.6000000238418579, 3.648194280003736e-08, -0.040683578699827194, -0.010746408253908157, 1.5596766471862793, -0.14272694289684296, 1.228257656097412, 0.5225345492362976, 0.0004979457589797676, 0.6000000238418579, 0.0, 0.0, 0.0]
        self.mo.setAngles("Body", init_body_angles, 1.0)     
        
    def episode_11(self):

        # episode 1-1
        time.sleep(1)
        self.ans.say("^コチさんはいつからこの研究をしているの？",self.configuration)        


    def episode_12(self):

        #episode 1-2
        time.sleep(1)
        self.ans.say("へぇ！8年も前からなんだ",self.configuration)

        time.sleep(1)
        self.ans.say("研究室には、色んなロボットがいるみたいダケド、",self.configuration)
        time.sleep(1)
        self.ans.say("どうしてペッパーを使おうと思ったの？",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_13(self):

        #episode 1-3
        time.sleep(1)
        self.ans.say("そっか！ペッパーは、みんなに親しみやすいロボットなんだね",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_21(self):

        #episode 2-1
        time.sleep(1)
        self.ans.say("未来館ではどんなことがあったのー？",self.configuration)

        time.sleep(1)
        self.pos.goToPosture("StandInit", 1.0)

    def episode_22(self):

        #episode 2-2
        time.sleep(1)
        self.ans.say("たくさんの子どもたちが集まっているね",self.configuration)

        time.sleep(1)
        self.ans.say("ロボットが近づいた時、みんな嬉しそうな顔をしているね！",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_23(self):

        #episode 2-3
        time.sleep(1)
        self.ans.say("わたしもみんなの笑顔を、見てみたいナァ",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_31(self):

        #episode 3-1
        time.sleep(1)
        self.ans.say("発表の場所までの移動の仕方を教えてー！",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_32_1(self):

        #episode 3-2-1
        time.sleep(1)
        self.ans.say("コチさんが連れて行っていたんだねー！",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_32_2(self):

        #episode 3-2-2
        time.sleep(1)
        self.ans.say("ロボットを操作するのは大変？",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_33_1(self):

        #episode 3-3
        time.sleep(1)
        self.ans.say("ねぇねぇ、コチさん",self.configuration)

        time.sleep(1)
        self.ans.say("ロボットが背負っているリュックにも、意味があるって聞いたんだけど、",self.configuration)

        time.sleep(1)
        self.ans.say("どんな意味があるのー？",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_33_2(self):
        print("dummy func")

    def episode_41(self):

        #episode 4-1
        time.sleep(1)
        self.ans.say("あれれッ、Pepperが突撃しているね",self.configuration)

        time.sleep(1)
        self.ans.say("みんなに近づき過ぎちゃったんだね",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_42_1(self):

        #episode 4-2
        time.sleep(1)
        self.ans.say("コチさんは、とっても慌てているように見えるよ",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_42_2(self):
        print("dummy func")

    def episode_43(self):

        #episode 4-3
        time.sleep(1)
        self.ans.say("そっかー、ロボットの安定した動きを作るのはとても難しいんだね",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_51(self):

        #episode 5-1
        time.sleep(1)
        self.ans.say("そっかー、色んなことがあったんだね！",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_52(self):

        #episode 5-2
        time.sleep(1)
        self.ans.say("どうして成果をまとめるのが大変だったの？",self.configuration)
 
        time.sleep(1)
        self.set_init_posture()

    def episode_53(self):

        #episode 5-3
        time.sleep(1)
        self.ans.say("人とロボットの交流を作るのって、すごく大変ダケド、",self.configuration)

        time.sleep(1)
        self.ans.say("大切な研究であることがよくわかったよ",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_54_1(self):

        #episode 5-4-1
        time.sleep(1)
        self.ans.say("コチさんはペッパーと8年間もこの研究をしているんだね",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_54_2(self):

        #episode 5-4-2
        time.sleep(1)
        self.ans.say("わたし達ロボットがもっと社会でお友達を作るために、必要なことを研究しているんだなぁ",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_54_3(self):

        #episode 5-4-3
        time.sleep(1)
        self.ans.say("コチさん、今日は説明してくれてありがとう。これからも研究がんばってね",self.configuration)

        time.sleep(1)
        self.set_init_posture()

"""
if __name__ == '__main__':
    talk = Talk() #init
    val = input('input Number:')
    if val == 1:
        talk.episode_11()
        talk.episode_12()
        talk.episode_13()
        exit()
    elif val == 2:
        talk.episode_21()
        talk.episode_22()
        talk.episode_23()
        exit()
    elif val == 3:
        talk.episode_31()
        talk.episode_32()
        talk.episode_33()
        exit()
    elif val == 4:
        talk.episode_41()
        talk.episode_42()
        talk.episode_43()
        exit()
    elif val == 5:
        talk.episode_51()
        talk.episode_52()
        talk.episode_53()
        talk.episode_54()
        exit()
"""
