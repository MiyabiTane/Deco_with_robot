#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from std_msgs.msg import Int32
from dialogflow_task_executive.msg import DialogResponse

class DialogflowSample(object):

    def __init__(self):
        self.pub = rospy.Publisher("/eyebrows/input_type", Int32, queue_size=1)
        rospy.Subscriber("/dialog_response", DialogResponse, self.dialog_cb)

    def dialog_cb(self, msg):
        print("action: ", msg.action)
        # print("query: ", msg.query)
        print("response: ", msg.response)
        print("fulfilled: ", msg.fulfilled)
        print("parameters: ", msg.parameters)
        print("speech_score: ", msg.speech_score)
        print("intent_score: ", msg.intent_score)
        # publish to eyebrows
        mode = msg.action
        pub_msg = Int32()
        action_to_int_dic = {"Happy": 1, "Relived": 2, "Smirking": 3, "Astonished": 4, "Cry": 5, "Fearful": 6,
                             "Flushed": 7, "Fearful": 8, "Love": 9, "Squinting": 10, "Boring": 11, "Cold_Sweat": 12}
        if mode in action_to_int_dic:
            pub_msg = action_to_int_dic[mode]
        else:
            pub_msg.data = 0
        self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node("dialogflow")
    dialogflowsample = DialogflowSample()
    rospy.spin()