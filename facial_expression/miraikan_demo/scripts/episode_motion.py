#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Code from https://github.com/a-ichikura/miraikan/blob/master/pepper_talk/episode_motion.py

import random
import rospy
from std_msgs.msg import String
import requests
import json
from naoqi import ALProxy
import time
import re
import almath

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
        # self.pos.goToPosture("StandInit", 1.0)
        self.set_init_posture()

        # open hands
        self.mo.openHand('LHand')
        self.mo.openHand('RHand')

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

    def set_init_posture_with_time(self, duration):
        init_body_angles = [0.0, -2.802596928649634e-45, 1.5596766471862793, 0.14272688329219818, -1.228257656097412, -0.5225345492362976, -0.000497947505209595, 0.6000000238418579, 3.648194280003736e-08, -0.040683578699827194, -0.010746408253908157, 1.5596766471862793, -0.14272694289684296, 1.228257656097412, 0.5225345492362976, 0.0004979457589797676, 0.6000000238418579, 0.0, 0.0, 0.0]
        self.mo.angleInterpolation("Body", init_body_angles, duration, True)

    def greeting(self):
        init_body_angles = [0.0, -2.802596928649634e-45,
                            1.5596766471862793, 0.14272688329219818, -1.228257656097412, -0.5225345492362976, -0.000497947505209595, 0.6000000238418579,
                            3.648194280003736e-08, -0.040683578699827194, -0.010746408253908157,
                            1.5596766471862793, -0.14272694289684296, 1.228257656097412, 0.5225345492362976, 0.0004979457589797676, 0.6000000238418579,
                            0.0, 0.0, 0.0]

        pose1 = [0.0, 0.0,
                 0.0, 10.0, -100.0, -70.0, 60.0, 0.0,
                 2.0, -2.0, -5.0,
                 0.0, -10.0, 100.0, 70.0, -60.0, 0.0,
                 0.0, 0.0, 0.0]
        pose1 = [n*almath.TO_RAD for n in pose1]

        pose2 = [0.0, 5.0,
                 0.0, 10.0, -110.0, -70.0, 60.0, 0.0,
                 2.0, -2.0, -5.0,
                 0.0, -10.0, 110.0, 70.0, -60.0, 0.0,
                 0.0, 0.0, 0.0]
        pose2 = [n*almath.TO_RAD for n in pose2]

        pose3 = [0.0, 5.0,
                 0.0, 10.0, -110.0, -70.0, 60.0, 0.0,
                 2.0, -2.0, -5.0,
                 0.0, -10.0, 110.0, 70.0, -60.0, 0.0,
                 0.0, 0.0, 0.0]
        pose3 = [n*almath.TO_RAD for n in pose3]

        self.mo.angleInterpolation("Body", pose1, 1.0, True)
        self.mo.angleInterpolation("Body", pose2, 1.0, True)
        self.mo.angleInterpolation("Body", pose1, 1.0, True)
        self.mo.angleInterpolation("Body", pose2, 1.0, True)
        self.mo.angleInterpolation("Body", pose1, 1.0, True)
        self.mo.angleInterpolation("Body", pose3, 1.0, True)
        self.mo.angleInterpolation("Body", init_body_angles, 3.0, True)

    def look_at_kochisan(self):
        self.set_init_posture()

        count = random.randrange(1,3)
        for i in range(count):
            self.mo.angleInterpolation(["HeadYaw", "HeadPitch"], [[-10.0*almath.TO_RAD, -10.0*almath.TO_RAD, -10.0*almath.TO_RAD], [0.0*almath.TO_RAD, -5.0*almath.TO_RAD, 0.0*almath.TO_RAD]], [[0.5, 1.0, 1.5], [0.5, 1.0, 1.5]], True)

        rDuration = 0.05
        self.led.post.fadeRGB( "FaceLed0", 0x000000, rDuration )
        self.led.post.fadeRGB( "FaceLed1", 0x000000, rDuration )
        self.led.post.fadeRGB( "FaceLed2", 0xffffff, rDuration )
        self.led.post.fadeRGB( "FaceLed3", 0x000000, rDuration )
        self.led.post.fadeRGB( "FaceLed4", 0x000000, rDuration )
        self.led.post.fadeRGB( "FaceLed5", 0x000000, rDuration )
        self.led.post.fadeRGB( "FaceLed6", 0xffffff, rDuration )
        self.led.fadeRGB( "FaceLed7", 0x000000, rDuration )
        time.sleep(0.1)
        self.led.fadeRGB( "FaceLeds", 0xffffff, rDuration )

        time.sleep(1.0)
        self.set_init_posture()

    def look_at_kochisan_mini(self):
        self.set_init_posture()
        self.mo.angleInterpolation(["HeadYaw", "HeadPitch"], [[-10.0*almath.TO_RAD, -10.0*almath.TO_RAD], [-5.0*almath.TO_RAD, -5.0*almath.TO_RAD]], [[1.0, 1.5], [1.0, 1.5]], True)
        rDuration = 0.05
        self.led.post.fadeRGB( "FaceLed0", 0x000000, rDuration )
        self.led.post.fadeRGB( "FaceLed1", 0x000000, rDuration )
        self.led.post.fadeRGB( "FaceLed2", 0xffffff, rDuration )
        self.led.post.fadeRGB( "FaceLed3", 0x000000, rDuration )
        self.led.post.fadeRGB( "FaceLed4", 0x000000, rDuration )
        self.led.post.fadeRGB( "FaceLed5", 0x000000, rDuration )
        self.led.post.fadeRGB( "FaceLed6", 0xffffff, rDuration )
        self.led.fadeRGB( "FaceLed7", 0x000000, rDuration )
        time.sleep(0.1)
        self.led.fadeRGB( "FaceLeds", 0xffffff, rDuration )
        self.mo.angleInterpolation(["HeadYaw", "HeadPitch"], [[0.0], [-2.802596928649634e-45]], [[1.0], [1.0]], True)

    def introduction(self):
        #introduction func
        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/Me_1)???????????????????????????????????????????????????",self.configuration)

        time.sleep(2)
        self.ans.say("^start(animations/Stand/Gestures/Explain_6)??????????????????????????????????????????",self.configuration)
        
        time.sleep(2)
        self.ans.say("?????????????????????????????????????????????^start(animations/Stand/Gestures/Explain_11)???????????????????????????",self.configuration)

        time.sleep(2)
        self.ans.say("????????????????????????????????????????????????^start(animations/Stand/Gestures/Hey_3)^wait(animations/Stand/Gestures/Hey_3)",self.configuration)

        time.sleep(1)
        self.set_init_posture()


    def episode_01(self):
        #episode 0-1                                                                                
        time.sleep(1)
        self.look_at_kochisan_mini()
        self.ans.say("????????????????????????????????????????????????^start(animations/Stand/Gestures/You_3)???????????????????????????????????????????????????",self.configuration)

    def episode_02(self):
        #episode 0-2                                                                                 
        time.sleep(1)
        self.look_at_kochisan_mini()
        self.mo.setStiffnesses(self.joint_names, 0.1)
        self.ans.say("???????????????^start(animations/Stand/Gestures/Me_1)?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????",self.configuration)
        self.set_init_posture_with_time(1.0)
        time.sleep(1.0)
        self.mo.setStiffnesses(self.joint_names, 1)

    def episode_11(self):

        # episode 1-1
        self.look_at_kochisan_mini()
        self.mo.setStiffnesses(self.joint_names, 0.5)
        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/You_1)?????????????????????????????????^wait(animations/Stand/Gesture/You_1)",self.configuration)

        self.mo.setStiffnesses(self.joint_names, 0.1)
        time.sleep(0.5)
        self.ans.say("^start(animations/Stand/Gestures/Enthusiastic_4)??????8???????????????^wait(animations/Stand/Gestures/Enthusiastic_4)",self.configuration)
        self.set_init_posture_with_time(1.0)
        time.sleep(1.0)
        self.mo.setStiffnesses(self.joint_names, 1)

    def episode_12(self):

        #episode 1-2
        time.sleep(1)
        self.ans.say("???????????????^start(animations/Stand/Gestures/Everything_3)????????????????????????????????????^wait(animations/Stand/Gesture/Everything_3)",self.configuration)

        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/Me_2)????????????????????????^wait(animations/Stand/Gestures/Me_2)",self.configuration)

        time.sleep(1)
        self.ans.say("????????????????????????????????????^start(animations/Stand/Gestures/Desperate_1)??????????????????^wait(animations/Stand/Gestures/Desperate_1)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_13(self):

        #episode 1-3
        time.sleep(1)
        self.ans.say("???????????????^start(animations/Stand/Gestures/Nothing_2)????????????????????????????????????^wait(animations/Stand/Gesture/Nothing_2)",self.configuration)

        self.look_at_kochisan_mini()
        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/Give_3)????????????????????????????????????????????????^wait(animations/Stand/Gestures/Give_3)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_14(self):

        #episode 1-4
        time.sleep(1)
        self.ans.say("3????????????????????????????????????????????????",self.configuration)

        time.sleep(1)
        self.ans.say("????????????????????????????????????^start(animations/Stand/Gestures/No_8)????????????????????????????????????",self.configuration)
        time.sleep(1)
        self.set_init_posture()

    def episode_21(self):

        #episode 2-1
        self.mo.setStiffnesses(self.joint_names, 0.1)
        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/Yes_1)?????????????????????????????????^wait(animations/Stand/Gestures/Yes_1)",self.configuration)

        self.mo.setStiffnesses(self.joint_names, 0.1)
        time.sleep(1)
        self.ans.say("^start(animations/Stand/Emotions/Positive/Happy_4)??????????????????????????????????????????^wait(animations/Stand/Emotions/Positive/Happy_4)",self.configuration)
        self.led.reset('FaceLeds')

        self.set_init_posture_with_time(1.0)
        time.sleep(1.0)
        self.mo.setStiffnesses(self.joint_names, 1)

    def episode_22(self):

        #episode 2-2
        self.look_at_kochisan_mini()
        time.sleep(1)
        self.ans.say("???????????????^start(animations/Stand/Gestures/You_4)????????????????????????????????????????????????????????????^wait(animations/Stand/Gesture/You_4)",self.configuration)

        time.sleep(1)
        self.ans.say("???????????????^start(animations/Stand/Gestures/ShowSky_7)?????????????????????????????????????????????????????????^wait(animations/Stand/Gestures/ShowSky_7)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_23(self):

        #episode 2-3
        time.sleep(1)
        self.ans.say("????????????????????????^start(animations/Stand/Gestures/ShowSky_1)?????????????????????????????????^wait(animations/Stand/Gesture/ShowSky_1)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_31(self):

        #episode 3-1
        self.look_at_kochisan_mini()
        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/Me_2)????????????????????????????????????^wait(animations/Stand/Gestures/Me_2)",self.configuration)

        time.sleep(1)
        self.ans.say("???????????????????????????^start(animations/Stand/Gestures/Give_4)?????????????????????????????????^wait(animations/Stand/Gestures/Give_4)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_32_1(self):

        #episode 3-2-1
        time.sleep(1)
        self.ans.say("???????????????????????????^start(animations/Stand/Gestures/IDontKnow_1)????????????????????????????????????^wait(animations/Stand/Gesture/IDontKnow_1)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_32_2(self):

        #episode 3-2-2
        self.look_at_kochisan_mini()
        time.sleep(1)
        self.ans.say("???????????????????????????????????????^start(animations/Stand/Gestures/ShowFloor_3)????????????????????????^wait(animations/Stand/Gestures/ShowFloor_3)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_33_1(self):

        #episode 3-3-1
        time.sleep(1)
        self.ans.say("??????????????????????????????????????????????????????^start(animations/Stand/Gestures/Hey_6)??????????????????????????????^wait(animations/Stand/Gesture/Hey_6)",self.configuration)
        
        time.sleep(1)
        self.set_init_posture()
    
    def episode_33_2(self):
        # episode 3-3-2
        self.mo.setStiffnesses(self.joint_names, 0.1)
        time.sleep(1)
        self.ans.say("??????????????????????????????^start(animations/Stand/Emotions/Positive/Happy_4)????????????????????????????????????^wait(animations/Stand/Emotions/Positive/Happy_4)",self.configuration)
        self.led.reset('FaceLeds')

        self.set_init_posture_with_time(1.0)
        time.sleep(1.0)
        self.mo.setStiffnesses(self.joint_names, 1)

    def episode_41(self):

        #episode 4-1
        self.mo.setStiffnesses(self.joint_names, 0.1)
        time.sleep(1)
        self.ans.say("???????????????????????????^start(animations/Stand/Gestures/Excited_1)???????????????????????????????????????^wait(animations/Stand/Gestures/Excited_1)",self.configuration)

        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/ShowSky_8)???????????????????????????????????????^wait(animations/Stand/Gestures/ShowSky_8)",self.configuration)

        self.set_init_posture_with_time(1.0)
        time.sleep(1.0)
        self.mo.setStiffnesses(self.joint_names, 1)

    def episode_42_1(self):

        #episode 4-2-1
        time.sleep(1.0)
        self.look_at_kochisan_mini()
        self.mo.setStiffnesses(self.joint_names, 0.1)
        time.sleep(1)
        self.ans.say("??????????????????^start(animations/Stand/Gestures/IDontKnow_2)??????????????????????????????^wait(animations/Stand/Gesture/IDontKnow_2)",self.configuration)
        self.set_init_posture_with_time(1.0)
        time.sleep(1.0)
        self.mo.setStiffnesses(self.joint_names, 1)

    def episode_42_2(self):
        #episode 4-2-2
        time.sleep(1)
        self.ans.say("???????????????????????????????????????????????????^start(animations/Stand/Gestures/Hey_3)????????????^wait(animations/Stand/Gestures/Hey_3)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_43(self):

        #episode 4-3
        self.look_at_kochisan_mini()
        time.sleep(1)
        self.ans.say("??????????????????????????????^start(animations/Stand/Gestures/Explain_6)??????????????????????????????????????????????????????^wait(animations/Stand/Gesture/Explain_6)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_51(self):

        #episode 5-1
        self.mo.setStiffnesses(self.joint_names, 0.1)
        time.sleep(1)
        self.ans.say("??????????????????^start(animations/Stand/Gestures/Yes_3)??????????????????????????????^wait(animations/Stand/Gestures/Yes_3)",self.configuration)
        self.set_init_posture_with_time(2.0)
        time.sleep(1.0)
        self.mo.setStiffnesses(self.joint_names, 1)

    def episode_52(self):

        #episode 5-2
        time.sleep(1)
        self.ans.say("??????????????????????????????^start(animations/Stand/Gestures/Everything_2)??????????????????????????????????????????^wait(animations/Stand/Gesture/Everything_2)",self.configuration)

        time.sleep(1)
        self.ans.say("??????????????????????????????????????????????????????^start(animations/Stand/Gestures/Thinking_1)????????????????????????????????????????????????^wait(animations/Stand/Gestures/Thinking_1)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_53(self):

        #episode 5-3
        self.look_at_kochisan_mini()
        time.sleep(1)
        self.ans.say("?????????????????????????????????????????????^start(animations/Stand/Gestures/No_8)?????????????????????????????????^wait(animations/Stand/Gesture/No_8)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_54_1(self):

        #episode 5-4-1
        self.look_at_kochisan_mini()
        time.sleep(1)
        self.ans.say("???????????????????????????^start(animations/Stand/Emotions/Positive/Peaceful_1)8?????????????????????????????????????????????????????????^wait(animations/Stand/Emotions/Positive/Peaceful_1)",self.configuration)
        self.led.reset('FaceLeds')

        time.sleep(1)
        self.set_init_posture()

    def episode_54_2(self):

        # episode 5-4-2
        self.look_at_kochisan_mini()
        time.sleep(1)
        self.ans.say("???????????????^start(animations/Stand/BodyTalk/BodyTalk_1)??????????????????????????????????????????^wait(animations/Stand/BodyTalk/BodyTalk_1)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def episode_54_3(self):

        #episode 5-4-3
        time.sleep(1)
        self.ans.say("????????????^start(animations/Stand/Gestures/Yes_3)???????????????????????????^wait(animations/Stand/Gestures/Yes_3)^start(animations/Stand/Gestures/Hey_3)?????????????????????????????????^wait(animations/Stand/Gestures/Hey_3)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def summary_1(self):

        #summary-1
        time.sleep(1)
        self.ans.say("???????????????",self.configuration)


        time.sleep(1)
        self.ans.say("?????????????????????????????????????????????????????????????????????????????????????????????????????????",self.configuration)

    def summary_2(self):

        #summary-2
        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/Explain_3)?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????",self.configuration)
    
    def summary_3(self):

        #summary-3
        time.sleep(1)
        self.ans.say("???????????????????????????????????????",self.configuration)

        time.sleep(1)
        self.ans.say("????????????????????????????????????^start(animations/Stand/Gestures/You_1)??????????????????",self.configuration)

        time.sleep(0.3)
        self.ans.say("^start(animations/Stand/Gestures/Give_4)?????????????????????????????????????????????????????????",self.configuration)

        time.sleep(0.3)
        self.ans.say("^start(animations/Stand/Gestures/Far_1)??????????????????????????????????????????????????????????????????",self.configuration)
        
        time.sleep(1)
        self.set_init_posture()

    def summary_4(self):

        #summary-4
        time.sleep(1)
        self.ans.say("??????????????????",self.configuration)
        
        time.sleep(0.3)
        self.ans.say("^start(animations/Stand/Gestures/Give_4)???????????????????????????^wait(animations/Stand/Gestures/Give_4)",self.configuration)

        time.sleep(0.3)
        self.ans.say("^start(animations/Stand/Gestures/Far_2)??????",self.configuration)

        time.sleep(0.3)
        self.ans.say("^start(animations/Stand/Gestures/ShowTablet_2)?????????????????????????????????????????????",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def summary_5(self):

        #summary-5
        time.sleep(1)
        self.ans.say("??????????????????",self.configuration)
        
        time.sleep(0.3)
        self.ans.say("^start(animations/Stand/Gestures/ShowSky_8)????????????????????????????????????????????????^wait(animations/Stand/Gestures/ShowSky_8)",self.configuration)

        time.sleep(1)
        self.set_init_posture()

    def summary_6(self):

        #summary-6
        time.sleep(1)
        self.ans.say("?????????????????????????????????????????????",self.configuration)
        
        time.sleep(0.3)
        self.ans.say("??????????????????????????????????????????????????????^start(animations/Stand/Gestures/Please_1)????????????????????????????????????",self.configuration)
        
        time.sleep(1)
        self.set_init_posture()

    def summary_7(self):

        #summary-7
        time.sleep(1)
        self.ans.say("^start(animations/Stand/Gestures/Everything_2)?????????????????????????????????????????????????????????",self.configuration)
        
        self.mo.setStiffnesses(self.joint_names, 0.1)
        time.sleep(0.3)
        self.ans.say("???????????????????????????^start(animations/Stand/Emotions/Positive/Peaceful_1)???????????????????????????????????????????????????^wait(animations/Stand/Emotions/Positive/Peaceful_1)",self.configuration)
        self.set_init_posture_with_time(2.0)
        time.sleep(1.0)
        self.mo.setStiffnesses(self.joint_names, 1)

    def summary_8(self):

        #summary-8
        time.sleep(1)
        self.ans.say("?????????????????????????????????????????????????????????????????????",self.configuration)
        
        time.sleep(0.3)
        self.ans.say("???????????????????????????????????????????????????????????????",self.configuration)

        time.sleep(0.3)
        self.mo.setStiffnesses(self.joint_names, 0.1)
        self.ans.say("^start(animations/Stand/Emotions/Positive/Peaceful_1)????????????????????????^wait(animations/Stand/Emotions/Positive/Peaceful_1)",self.configuration)
        self.led.reset('FaceLeds')

        self.set_init_posture_with_time(1.0)
        time.sleep(1.0)
        self.mo.setStiffnesses(self.joint_names, 1)


"""
if __name__ == '__main__':
    talk = Talk("169.254.175.13") #init
    while(True):
        val = input('input Number:')
        if val == 0:
            talk.look_at_kochisan_mini()
        elif val == 1:
            talk.greeting()
        elif val == 2:
            talk.look_at_kochisan()
        if val == 3:
            talk.introduction()
        elif val == 4:
            talk.episode_01()
            time.sleep(5.0)
            talk.episode_02()
        elif val == 5:
            talk.episode_11()
            time.sleep(3.0)
            talk.episode_12()
            time.sleep(10.0)
            talk.episode_13()
        elif val == 6:
            talk.episode_14()
        elif val == 7:
            talk.episode_21()
            talk.episode_22()
            talk.episode_23()
        elif val == 8:
            talk.episode_31()
            time.sleep(5.0)
            talk.episode_32_1()
            talk.episode_32_2()
            time.sleep(3.0)
            talk.episode_33_1()
            talk.episode_33_2()
        elif val == 9:
            talk.episode_41()
            talk.episode_42_1()
            time.sleep(2.0)
            talk.episode_42_2()
            talk.episode_43()
        elif val == 10:
            talk.episode_51()
            talk.episode_52()
            time.sleep(10.0)
            talk.episode_53()
            talk.episode_54_1()
        elif val == 11:
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
