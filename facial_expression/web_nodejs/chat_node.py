#!/usr/bin/env python
# -*- coding: utf-8 -*-

# input: /speech_to_text, output: /robotsound_jp
import rospy
import sys
import argparse
import json
import requests
import actionlib

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sound_play.msg import SoundRequestAction, SoundRequestGoal

if sys.version_info.major == 2:
    reload(sys)
    sys.setdefaultencoding('utf-8')

class ChaplusRos(object):

    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--path", required=True)
        args = parser.parse_args()
        print(args.path)
        try:
            with open(args.path) as j:
                apikey_json = json.loads(j.read())
            with open("chaplus_info.json") as j:
                self.data_json = json.loads(j.read())
            self.url = 'https://www.chaplus.jp/v1/chat?apikey={}'.format(apikey_json['apikey'])
        except:
            rospy.logerr("Cannnot open json files")
        self.headers = {'content-type': 'text/json'}

        self.volume = 1.0
        self.command = 1
        self.sound = -3
        self.arg2 = 'ja'

        self.sub = rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, self.chat_cb)
        self.actionlib_client = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.actionlib_client.wait_for_server()

    def chat_response(self, input_text):
        best_response = ""
        try:
            rospy.loginfo("received {}".format(input_text))
            # self.data = json.dumps({'utterance': msg.data})
            self.data_json["utterance"] = input_text
            self.data = json.dumps(self.data_json)
            response = requests.post(
                url=self.url, headers=self.headers, data=self.data)
            response_json = response.json()
            if not response_json.has_key('bestResponse'):
                best_response = "ごめんなさい、よくわからないです"
            else:
                best_response = response_json['bestResponse']['utterance']
        except Exception as e:
            rospy.logerr("Failed to reqeust url={}, headers={}, data={}".format(
                self.url, self.headers, self.data))
            rospy.logerr(e)
            best_response = "ごめんなさい、よくわからないです"
        rospy.loginfo("chaplus: returns best response {}".format(best_response))
        return best_response

    def chat_cb(self, msg):
        listen_text = ""
        if msg.transcript:
            for char in msg.transcript:
                listen_text += char
        response_text = self.chat_response(listen_text)
        speak_msg = SoundRequestGoal()
        speak_msg.sound_request.volume = self.volume
        speak_msg.sound_request.command = self.command
        speak_msg.sound_request.sound = self.sound
        speak_msg.sound_request.arg = response_text
        speak_msg.sound_request.arg2 = self.arg2
        self.actionlib_client.send_goal(speak_msg)


if __name__ == '__main__':
    rospy.init_node("chaplus_ros")
    chaplus_ros = ChaplusRos()
    rospy.spin()
