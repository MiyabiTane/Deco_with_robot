#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import subprocess
from std_msgs.msg import Int32
from std_msgs.msg import String
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
        # pub_msg = Int32()
        pub_str_int = ""
        action_to_int_dic = {"Happy": 1, "Relived": 2, "Smirking": 3, "Astonished": 4, "Unpleasant": 5, "Angry": 6,
                             "Flushed": 7, "Fearful": 8, "Love": 9, "Squinting": 10, "Boring": 11, "Cry": 12}
        if mode in action_to_int_dic:
            if msg.intent_score > 0.60:
                pub_str_int = str(action_to_int_dic[mode])
                # pub_msg = action_to_int_dic[mode]
        else:
            pub_str_int = "0"
            # pub_msg.data = 0
        # 音声と表情の時差をなくしたいので一度デフォルトの表情に戻す
        subprocess.call(["rostopic", "pub", "-1", "/eyebrows/input_type", "std_msgs/Int32", "data: 0"])

        subprocess.call(["rostopic", "pub", "-1", "/text_for_sound", "std_msgs/String", "data: " + msg.query])
        subprocess.call(["rostopic", "pub", "-1", "/eyebrows/input_type", "std_msgs/Int32", "data: " + pub_str_int])
        # self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node("dialogflow")
    dialogflowsample = DialogflowSample()
    rospy.spin()