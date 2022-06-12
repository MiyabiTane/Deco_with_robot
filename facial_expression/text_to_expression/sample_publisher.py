#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess

from std_msgs.msg import String
from dialogflow_task_executive.msg import DialogResponse

sample_sentences = [ "素敵な一日を過ごしてね", "一緒に出来て嬉しいな", "楽しいです",  # Happy😀
                     "お疲れ様です", "今日はいいことがあったの", "なんだか落ち着きます",  # Relieved😌
                     "ぶっちゃけ予想通りだわ", # Smirking😏
                     "知らなかったびっくり", "驚きました", "凄いですね", # Astonished😲
                     "悲しいです", "いい話ですね、感動した", "そんなこと言われたら、もう泣いちゃう", # Cry😭
                     "そういうのやめなよ", "ふざけないで", "怒りますよ", # Angry😠
                     "きゃー恥ずかしい", "照れちゃう", "顔が熱いです", # Flushed😳
                     "そんなことが起こるなんて！", "怖い...", "恐ろしいエピソードだ", # Fearful😱
                     "かわいいですよね", "癒やされます", "私もそれが好きです", # Love😍
                     "私も同じこと思いました", "どうだ、騙されたか〜", "思い通りじゃね", # Squinting😝
                     "おやすみなさい", "ふーん", "心地良いなぁ", # Boring😪
                     "ひどいこと言うんですね", "私には難しいです", "痛そうですね" # Unpleasant😓
                    ]


unexpected_sentences = ["怪我がなくてよかったよ", "仲よしで微笑ましいな", "そんなものがあるのですね",
                        "え、ちょっとまって！", "え、ちょっとまって意外なんだけど",
                        "怒った", "ムカつくんだけど", "みんなの成長が微笑ましいですね", "微笑ましいですね",
                        "痛そう、大丈夫ですか？", "みんな仲良くしようよ", "思い通りだわ", "思い通りじゃね",
                        "怒らないで", "心地よいなぁ", "心地良いなぁ", "私も会ってみたい", "あなたが気になります",
                        "素直なんだなぁ", "なんと斬新な！", "つい笑っちゃう", "なんで出来ないのでしょう",
                        "気持ち悪いです", "それ、気持ち悪いです", "気持ち悪い", "馬鹿にするな",
                        "得体がしれないです", "恐ろしいエピソードだ", "恐ろしいエピソードですね"]

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
        rospy.sleep(4)
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