#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess

from std_msgs.msg import String
from dialogflow_task_executive.msg import DialogResponse

sample_sentences = [ "今日はいいことがあったの", "素敵な一日を過ごしてね", "一緒に出来て嬉しいな", # Happy😀
                     "お疲れ様です", "心地よいなぁ", "楽しいです",  # Relieved😌
                     "素直なんだなぁ", # Smirking😏
                     "知らなかったびっくり", "驚きました", "凄いですね", # Astonished😲
                     "悲しいです", "いい話ですね、感動した！", "そんなこと言われたら、もう泣いちゃう", # Cry😭
                     "ばかにするな", "そういうのやめなよ", "ふざけないで", # Angry😠
                     "きゃー恥ずかしい", "私も会ってみたい", "あなたが気になります", # Flushed😳
                     "そんなことが起こるなんて！", "怖い...", "痛そうですね", # Fearful😱
                     "かわいいですよね", "癒やされます", "私もそれが好きです", # Love😍
                     "なんと斬新な！", "つい笑っちゃう", "それは面白いエピソードだ",  # Squinting😝
                     "もう眠いです", "おやすみなさい", "ふーん", # Boring😪
                     "ひどいこと言うんだなぁ", "私には難しいです", "なんで出来ないのでしょう" # Cold_Sweat😓
                    ] 

unexpected_sentences = ["怪我がなくてよかったよ", "仲よしで微笑ましいな", "そんなものがあるのですね",
                        "え、ちょっとまって！", "え、ちょっとまって意外なんだけど", "ひどいこと言うんですね",
                        "怒った", "ムカつくんだけど", "みんなの成長が微笑ましいですね", "微笑ましいですね",
                        "私も同じこと思いました", "痛そう、大丈夫ですか？", "ひどいこと言うんですね", "みんな仲良くしようよ",
                        "怒らないで"]

EXAMPLE_SENTENCES = sample_sentences + unexpected_sentences

class FacialExpressionSample(object):

    def __init__(self):
        self.sample_words = EXAMPLE_SENTENCES
        self.published_words = self.sample_words[0]
        self.pub = rospy.Publisher("/text", String, queue_size=1)
        rospy.Subscriber("/dialog_response", DialogResponse, self.expression_cb)
    
    def pub_sentence(self):
        if len(self.sample_words) > 0:
            pub_msg = String()
            pub_words = self.sample_words.pop(0)
            pub_msg.data = pub_words
            self.pub.publish(pub_msg)
            self.published_words = pub_words
    
    def expression_cb(self, msg):
        print(self.published_words)
        print("===")
        print("Action: {}, reliability: {}".format(msg.action, msg.intent_score))
        print("===")
        rospy.sleep(5)
        self.pub_sentence()
    
    def pub_sentence_first(self):
        if len(self.sample_words) > 0:
            pub_words = self.sample_words.pop(0)
            subprocess.call(["rostopic", "pub", "-1", "/text", "std_msgs/String", "data: " + pub_words])


if __name__ == '__main__':
    rospy.init_node("dialogflow")
    facial_expression_sample = FacialExpressionSample()
    facial_expression_sample.pub_sentence_first()
    rospy.spin()