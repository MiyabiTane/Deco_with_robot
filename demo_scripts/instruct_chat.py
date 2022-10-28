#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib

from std_msgs.msg import String
from ros_google_cloud_language.msg import AnalyzeTextAction, AnalyzeTextGoal

class InstructChat(object):
    def __init__(self):
        self.direction = ""
        self.action_flag = False
        self.response = ""

        self.set_words()

        self.actionlib_client = actionlib.SimpleActionClient("/analyze_text/text", AnalyzeTextAction)
        self.actionlib_client.wait_for_server()

        rospy.Subscriber("/input_gcl_text", String, self.gcl_cb)
        # service callにする。方向をリセットするかの情報も送る

    def set_words(self):
        ambiguous_direction_lst = ["そっち", "あっち", "こっち"]
        self.ambiguous_place_lst = ["そこ", "ここ", "あそこ"]
        self.clear_direction_lst = ["右", "左", "上", "下"]
        degree_lst = ["もっと", "少し", "ちょっと"]
        indicator_lst = ["その", "あの", "この"]
        ambiguous_object_lst = ["それ", "これ", "あれ"]
        clear_object_lst = ["飾り", "風船"]
        denial_lst = ["ない", "違う"]
        self.action_lst = ["置い", "動かし", "ずらし", "する"]
        self.suggestion_lst = ["良い", "ほしい", "いい"]
        self.hot_word_lst = clear_object_lst + ambiguous_object_lst + degree_lst + ambiguous_direction_lst

    def gcl_cb(self, msg):
        # for debug
        if msg.data == "":
            self.direction = ""
        self.action_flag = False
        self.response = ""
        result = self.get_parsing_result(msg.data)
        lemma_lst = []
        for syntax in result.syntaxes:
            lemma_lst.append(syntax.lemma)
        for lemma_word in lemma_lst:
            # 上下左右
            for direction in self.clear_direction_lst:
                if lemma_word == direction:
                    if not self.is_head_index_instruct(result.syntaxes, lemma_lst, lemma_word):
                        self.direction = lemma_word
                        self.action_flag = True
            # お願いする形容詞
            if not self.action_flag:
                for suggestion in self.suggestion_lst:
                    if lemma_word == suggestion:
                        if self.is_head_index_instruct(result.syntaxes, lemma_lst, suggestion):
                            self.action_flag = True
            # 指示に関するその他のワード
            if not self.action_flag:
                for hot_word in self.hot_word_lst:
                    if lemma_word == hot_word:
                        self.action_flag = True
            # 曖昧な指示方向
            if not self.action_flag:
                for a_direction in self.ambiguous_place_lst:
                    if lemma_word == a_direction:
                        self.response = "ごめんなさい。たくさんは動かせないです"
        if not self.response:
            if not self.action_flag:
                self.response = "指示モード終了"
            elif self.direction:
                self.response = "風船を" + self.direction + "に動かします"
            else:
                self.response = "上下左右どちらに動かしたら良いですか？"
        print(msg.data)
        print("-> " + self.response)

    def is_head_index_instruct(self, syntaxes, lemma_lst, word):
        word_pos = lemma_lst.index(word)
        for syntax in syntaxes:
            if syntax.dependency_edge == word_pos:
                if syntax.lemma in self.action_lst:
                    return True
        return False

    def is_head_index_neg(self, syntaxes, lemma_lst, word):
        word_pos = lemma_lst.index(word)
        for syntax in syntaxes:
            if syntax.dependency_edge == word_pos:
                if syntax.parse_label == 25:
                    return True
        return False
    
    def get_parsing_result(self, input_sentence):
        pub_action_msg = AnalyzeTextGoal()
        pub_action_msg.text = input_sentence
        self.actionlib_client.send_goal(pub_action_msg)
        self.actionlib_client.wait_for_result()
        result = self.actionlib_client.get_result()
        return result

    def print_parsing_result(self, input_sentence):
        result = self.get_parsing_result(input_sentence)
        entity_dict = {0: "UNKNOWN", 1: "PERSON", 2: "LOCATION", 3: " ORGANIZATION", 4: "EVENT",
                        5: "WORK_OF_ART", 6: "CONSUMER_GOOD", 7: "OTHER", 9: "PHONE_NUMBER",
                        10: "ADDRESS", 11: "DATE", 12: "NUMBER", 13: "PRICE"}
        syntax_dict = {0: "UNKNOWN", 1: "ADJ", 2: "ADP", 3: "ADV", 4: "CONJ", 5: "DET", 6: "NOUN",
                        7: "NUM", 8: "PRON", 9: "PRT", 10: "PUNCT", 11: "VERB", 12: "X", 13: "AFFIX"}
        en_to_jp_dict = {"UNKNOWN": "不明", "ADJ": "形容詞", "ADP": "設置詞", "ADV": "副詞",
                         "CONJ": "接続詞", "DET": "限定詞", "NOUN": "名詞", "NUM": "基数", "PRON": "代名詞",
                         "PRT": "その他", "PUNCT": "句読点", "VERB": "動詞", "X": "不明", "AFFIX": "接辞"}
        # parse_label: 25 "否定"
        print("== Entity Info")
        for entity in result.entities:
            e_type = entity_dict[entity.type]
            print("{}, type: {}" .format(entity.name, e_type))
        print("\n== Syntax Info")
        for syntax in result.syntaxes:
            depend_word = result.syntaxes[syntax.dependency_edge].name
            s_part = en_to_jp_dict[syntax_dict[syntax.part_of_speech]]
            print("{}, lemma: {}, depend_word: {}, part: {}, label: {}"
                    .format(syntax.name, syntax.lemma, depend_word, s_part, syntax.parse_label))


if __name__ == '__main__':
    rospy.init_node("instruct_chat")
    instruct_chat = InstructChat()
    # instruct_chat.print_parsing_result("その風船はそっちじゃなくてもっと右に置いてほしい")
    rospy.spin()
