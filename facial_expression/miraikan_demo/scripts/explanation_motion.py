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
        try:
            self.tts = ALProxy("ALTextToSpeech",self.PEPPER_IP,self.PORT)
        except Exception, e:
            print "Error:"
            print str(e)

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

        try:
            self.led = ALProxy("ALLeds",self.PEPPER_IP,self.PORT)
        except Exception, e:
            print "Error:"
            print str(e)

        #set init posture                                                                           
        self.set_init_posture()

        
        #set init speech setting
        self.tts.setLanguage("Japanese")
        self.tts.setParameter("pitch", 1.3)
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
        self.mo.setAngles("Body", init_body_angles, 0.1)     

    def introduction(self):
        #introduction func     
        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/Me_1)私は、人型ロボットのペッパーです。",self.configuration)

        time.sleep(2)
        self.ans.say("^start(animations/Stand/Gestures/Explain_6)名前は、ホップって言うよ。",self.configuration)

        time.sleep(2)
        self.ans.say("今日は、司会として^start(animations/Stand/Gestures/Explain_11)コチさんの発表をお手伝いするよ",self.configuration)

        time.sleep(2)
        self.ans.say("みんな、今日はどうぞよろしくね。^start(animations/Stand/Gestures/Hey_3)^wait(animations/Stand/Gestures/Hey_3)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_01(self):
        #episode 0-1
        time.sleep(1)
        self.ans.say("^コチさん、どうしてこの研究を始めたのー？",self.configuration)

    def episode_02(self):
        #episode 0-2
        time.sleep(1)
        self.ans.say("^へー、ソウなんだ。知らない人と話すのって緊張するもんね。",self.configuration)

        time.sleep(1)
        self.set_init_posture()       
        
    def episode_11(self):

        # episode 1-1
        time.sleep(1)
        self.ans.say("^コチさんはいつからこの研究をしているの？",self.configuration)        


    def episode_12(self):

        #episode 1-2
        time.sleep(1)
        self.ans.say("ソウなんだね^start(animations/Stand/Gestures/Yes_2)",self.configuration)
  

        time.sleep(3)
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

    def episode_14(self):

        #episode 1-4
        time.sleep(1)
        self.ans.say("すごい！３つのポイント、わかりやすいね",self.configuration)

        time.sleep(1)
        self.set_init_posture()
        
        
    def episode_21(self):

        #episode 2-1
        print("dummy func")

    def episode_22(self):

        #episode 2-2
        time.sleep(1)
        self.ans.say("ペッパーの手を、とってくれているコがいるね.",self.configuration)

        time.sleep(1)
        self.ans.say("みんな、嬉しそうな様子だね.",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_23(self):

        #episode 2-3
        print("dummy func")

    def episode_31(self):

        #episode 3-1
        time.sleep(1)
        self.ans.say("発表の場所までの移動の仕方を教えて！",self.configuration)

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
        print("dummy func")


    def episode_33_1(self):

        #episode 3-3
        time.sleep(1)
        self.ans.say("ねぇねぇ、コチさん",self.configuration)

        time.sleep(1)
        self.ans.say("ペッパーは、どうしてリュックを背負っているのー？",self.configuration)

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
        self.ans.say("どうして大変だったの？",self.configuration)
 
        time.sleep(1)
        self.set_init_posture()

    def episode_53(self):

        #episode 5-3
        time.sleep(1)
        self.ans.say("そっかー、それは大変だったね。",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_54_1(self):

        #episode 5-4-1
        time.sleep(1)
        self.ans.say("でもコチさん、８年間たくさん頑張ったんだね。",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_54_2(self):

        #episode 5-4-2
        self.mo.setStiffnesses(self.joint_names, 1)
        time.sleep(1)
        self.ans.say("コチさん、^start(animations/Stand/BodyTalk/BodyTalk_1)博士の卒業、本当におめでとう^wait(animations/Stand/BodyTalk/BodyTalk_1)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_54_3(self):

        #episode 5-4-3        
        time.sleep(1)
        self.ans.say("今日は説明してくれてありがとう。これからも研究がんばってね",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def summary_1(self):

        #summary-1                                                                                                                                                                                          
        self.mo.setStiffnesses(self.joint_names, 1)
        time.sleep(1)
        self.ans.say("わかったー",self.configuration)


        time.sleep(1)
        self.mo.setStiffnesses(self.joint_names, 1)
        self.ans.say("今日は、ロボットによる人同士の交流づくりについて、コチさんとお話したよ",self.configuration)

    def summary_2(self):

        #summary-2                                                                                                                                                                                          
        self.mo.setStiffnesses(self.joint_names, 1)
        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/Explain_3)まず、コチさんの夢、コチさんが研究を始めたきっかけ、ペッパーを使うようになった理由を話したね。",self.configuration)

    def summary_3(self):

        #summary-3                                                                                                                                                                                          
        self.mo.setStiffnesses(self.joint_names, 1)
        time.sleep(1)
        self.ans.say("次に、研究のポイントだね。",self.configuration)

        time.sleep(1)
        self.ans.say("交流づくりのポイントは、^start(animations/Stand/Gestures/You_1)接近すること",self.configuration)

        time.sleep(0.3)
        self.ans.say("^start(animations/Stand/Gestures/Give_4)人から世話を引き出すロボットにすること",self.configuration)

        time.sleep(0.3)
        self.ans.say("^start(animations/Stand/Gestures/Far_1)研究室を飛び出して科学館で研究したことだよ。",self.configuration)

        time.sleep(1)
        self.set_init_posture()


    def summary_4(self):

        #summary-4                                                                                                                                                                                          
        self.mo.setStiffnesses(self.joint_names, 1)
        time.sleep(1)
        self.ans.say("交流のこつは",self.configuration)

        time.sleep(0.3)
	self.ans.say("^start(animations/Stand/Gestures/Give_4)共感を引き出すこと^wait(animations/Stand/Gestures/Give_4)",self.configuration)

        time.sleep(0.3)
	self.ans.say("^start(animations/Stand/Gestures/Far_2)移動",self.configuration)

	time.sleep(0.3)
        self.ans.say("^start(animations/Stand/Gestures/ShowTablet_2)触ってもらうことだとわかったよ",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def summary_5(self):

        #summary-5                                                                                                                                                                                          
        self.mo.setStiffnesses(self.joint_names, 1)
        time.sleep(1)
        self.ans.say("発表の途中で",self.configuration)

        time.sleep(0.3)
        self.ans.say("^start(animations/Stand/Gestures/ShowSky_8)客席に突進しちゃうこともあったね^wait(animations/Stand/Gestures/ShowSky_8)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def summary_6(self):

        #summary-6                                                                                                                                                                                          
        self.mo.setStiffnesses(self.joint_names, 1)
        time.sleep(1)
        self.ans.say("初めて誰かとお話しする場面で、",self.configuration)

        time.sleep(0.3)
        self.ans.say("ロボットにどんなことをしてほしいか、^start(animations/Stand/Gestures/Please_1)みんなの意見も紹介したよ",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def summary_7(self):

        #summary-7                                                                                  
        self.mo.setStiffnesses(self.joint_names, 1)
        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/Everything_2)他にも色んなたいへんなことがあったけど",self.configuration)

        self.mo.setStiffnesses(self.joint_names, 0)
        time.sleep(0.3)
        self.ans.say("みんなのおかげで、^start(animations/Stand/Emotions/Positive/Peaceful_1)無事に博士論文をまとめられたんだね^wait(animations/Stand/Emotions/Positive/Peaceful_1)",self.configuration)   
        time.sleep(1)
        self.set_init_posture()

    def summary_8(self):

        #summary-8                                                                       
        self.mo.setStiffnesses(self.joint_names, 1)
        time.sleep(1)
        self.ans.say("この研究は、人の交流を手助けできるだけでなく、",self.configuration)

        time.sleep(0.3)
        self.ans.say("人とロボットがナカよくなっていくためにも、",self.configuration)

        time.sleep(0.3)
        self.mo.setStiffnesses(self.joint_names, 0)
        self.ans.say("^start(animations/Stand/Emotions/Positive/Peaceful_1)大切な研究だね。^wait(animations/Stand/Emotions/Positive/Peaceful_1)",self.configuration)
        self.led.off('FaceLeds')
        self.led.off('FaceLedsInternal')

        time.sleep(1)
        self.mo.setStiffnesses(self.joint_names, 1)
        self.set_init_posture()


"""
if __name__ == '__main__':
    talk = Talk("169.254.175.13") #init
    while(True):
        val = input('input Number:')
        if val == 1:
            talk.introduction()
        elif val == 2:
            talk.episode_01()
            time.sleep(5.0)
            talk.episode_02()
        elif val == 3:
            talk.episode_11()
            time.sleep(3.0)
            talk.episode_12()
            time.sleep(10.0)
            talk.episode_13()
        elif val == 4:
            talk.episode_14()
        elif val == 5:
            talk.episode_21()
            talk.episode_22()
            talk.episode_23()
        elif val == 6:
            talk.episode_31()
            time.sleep(5.0)
            talk.episode_32_1()
            talk.episode_32_2()
            time.sleep(3.0)
            talk.episode_33_1()
            talk.episode_33_2()
        elif val == 7:
            talk.episode_41()
            talk.episode_42_1()
            time.sleep(2.0)
            talk.episode_42_2()
            talk.episode_43()
        elif val == 8:
            talk.episode_51()
            talk.episode_52()
            time.sleep(10.0)
            talk.episode_53()
            talk.episode_54_1()
        elif val == 9:
            talk.summary_1()
            talk.summary_2()
            talk.summary_3()
            talk.summary_4()
            talk.summary_5()
            talk.summary_6()
            talk.summary_7()
            talk.summary_8()
            time.sleep(3.0)
            talk.episode_54_2()
            talk.episode_54_3()
"""
