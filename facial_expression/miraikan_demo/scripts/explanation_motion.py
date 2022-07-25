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
        
    def explanation_11(self):

        # episode 1-1
        time.sleep(1)
        self.ans.say("^コチさんはいつからこの研究をしているの？",self.configuration)        


    def explanation_12(self):

        #explanation 1-2
        time.sleep(1)
        self.ans.say("へぇ！8年も前からなんだ",self.configuration)

        time.sleep(1)
        self.ans.say("研究室には、色んなロボットがいるみたいダケド、",self.configuration)
        time.sleep(1)
        self.ans.say("どうしてペッパーを使おうと思ったの？",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def explanation_13(self):

        #explanation 1-3
        time.sleep(1)
        self.ans.say("そっか！ペッパーは、みんなに親しみやすいロボットなんだね",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def explanation_21(self):

        #explanation 2-1
        time.sleep(1)
        self.ans.say("未来館ではどんなことがあったのー？",self.configuration)

        time.sleep(1)
        self.pos.goToPosture("StandInit", 1.0)

    def explanation_22(self):

        #explanation 2-2
        time.sleep(1)
        self.ans.say("たくさんの子どもたちが集まっているね",self.configuration)

        time.sleep(1)
        self.ans.say("ロボットが近づいた時、みんな嬉しそうな顔をしているね！",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def explanation_23(self):

        #explanation 2-3
        time.sleep(1)
        self.ans.say("わたしもみんなの笑顔を、見てみたいナァ",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def explanation_31(self):

        #explanation 3-1
        time.sleep(1)
        self.ans.say("発表の場所までの移動の仕方を教えてー！",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def explanation_32(self):

        #explanation 3-2
        time.sleep(1)
        self.ans.say("コチさんが連れて行っていたんだねー！",self.configuration)

        time.sleep(1)
        self.ans.say("ロボットを操作するのは大変？",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def explanation_33(self):

        #explanation 3-3
        time.sleep(1)
        self.ans.say("ねぇねぇ、コチさん",self.configuration)

        time.sleep(1)
        self.ans.say("ロボットが背負っているリュックにも、意味があるって聞いたんだけど、",self.configuration)

        time.sleep(1)
        self.ans.say("どんな意味があるのー？",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def explanation_41(self):

        #explanation 4-1
        time.sleep(1)
        self.ans.say("あれれッ、Pepperが突撃しているね",self.configuration)

        time.sleep(1)
        self.ans.say("みんなに近づき過ぎちゃったんだね",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def explanation_42(self):

        #explanation 4-2
        time.sleep(1)
        self.ans.say("コチさんは、とっても慌てているように見えるよ",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def explanation_43(self):

        #explanation 4-3
        time.sleep(1)
        self.ans.say("そっかー、ロボットの安定した動きを作るのはとても難しいんだね",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def explanation_51(self):

        #explanation 5-1
        time.sleep(1)
        self.ans.say("そっかー、色んなことがあったんだね！",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def explanation_52(self):

        #explanation 5-2
        time.sleep(1)
        self.ans.say("どうして成果をまとめるのが大変だったの？",self.configuration)
 
        time.sleep(1)
        self.set_init_posture()

    def explanation_53(self):

        #explanation 5-3
        time.sleep(1)
        self.ans.say("人とロボットの交流を作るのって、すごく大変ダケド、",self.configuration)

        time.sleep(1)
        self.ans.say("大切な研究であることがよくわかったよ",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def explanation_54(self):

        #explanation 5-4
        time.sleep(1)
        self.ans.say("コチさんはペッパーと8年間もこの研究をしているんだね",self.configuration)

        time.sleep(1)
        self.ans.say("わたし達ロボットがもっと社会でお友達を作るために、必要なことを研究しているんだなぁ",self.configuration)

        time.sleep(1)
        self.ans.say("コチさん、今日は説明してくれてありがとう。これからも研究がんばってね",self.configuration)

        time.sleep(1)
        self.set_init_posture()

if __name__ == '__main__':
    talk = Talk() #init
    val = input('input Number:')
    if val == 1:
        talk.explanation_11()
        talk.explanation_12()
        talk.explanation_13()
        exit()
    elif val == 2:
        talk.explanation_21()
        talk.explanation_22()
        talk.explanation_23()
        exit()
    elif val == 3:
        talk.explanation_31()
        talk.explanation_32()
        talk.explanation_33()
        exit()
    elif val == 4:
        talk.explanation_41()
        talk.explanation_42()
        talk.explanation_43()
        exit()
    elif val == 5:
        talk.explanation_51()
        talk.explanation_52()
        talk.explanation_53()
        talk.explanation_54()
        exit()


        
        
