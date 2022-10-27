#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib

from ros_google_cloud_language.msg import AnalyzeTextAction, AnalyzeTextGoal

class InstructChat(object):
    def __init__(self):
        self.actionlib_client = actionlib.SimpleActionClient("/analyze_text/text", AnalyzeTextAction)
        self.actionlib_client.wait_for_server()
    
    def print_parsing_result(self, input_sentence):
        entity_dict = {0: "UNKNOWN", 1: "PERSON", 2: "LOCATION", 3: " ORGANIZATION", 4: "EVENT",
                        5: "WORK_OF_ART", 6: "CONSUMER_GOOD", 7: "OTHER", 9: "PHONE_NUMBER",
                        10: "ADDRESS", 11: "DATE", 12: "NUMBER", 13: "PRICE"}
        syntax_dict = {0: "UNKNOWN", 1: "ADJ", 2: "ADP", 3: "ADV", 4: "CONJ", 5: "DET", 6: "NOUN",
                        7: "NUM", 8: "PRON", 9: "PRT", 10: "PUNCT", 11: "VERB", 12: "X", 13: "AFFIX"}
        en_to_jp_dict = {"UNKNOWN": "不明", "ADJ": "形容詞", "ADP": "設置詞", "ADV": "副詞",
                         "CONJ": "接続詞", "DET": "限定詞", "NOUN": "名詞", "NUM": "基数", "PRON": "代名詞",
                         "PRT": "その他", "PUNCT": "句読点", "VERB": "動詞", "X": "不明", "AFFIX": "接辞"}
        pub_action_msg = AnalyzeTextGoal()
        pub_action_msg.text = input_sentence
        self.actionlib_client.send_goal(pub_action_msg)
        self.actionlib_client.wait_for_result()
        result = self.actionlib_client.get_result()
        print("== Entity Info")
        for entity in result.entities:
            e_type = entity_dict[entity.type]
            print("{}, type: {}" .format(entity.name, e_type))
        print("\n== Syntax Info")
        for syntax in result.syntaxes:
            depend_word = result.syntaxes[syntax.dependency_edge].name
            s_part = en_to_jp_dict[syntax_dict[syntax.part_of_speech]]
            print("{}, depend_word: {}, part: {}" .format(syntax.name, depend_word, s_part))


if __name__ == '__main__':
    rospy.init_node("instruct_chat")
    instruct_chat = InstructChat()
    instruct_chat.print_parsing_result("その風船はもっと右に置いてほしい")
