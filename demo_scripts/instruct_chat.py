#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib

from std_msgs.msg import String
from ros_google_cloud_language.msg import AnalyzeTextAction, AnalyzeTextGoal

# self.direction_up, self.direction_down, self.direction_left, self.direction_right = 0, 0, 0, 0

class InstructChat(object):
    def __init__(self):
        self.direction_up = 0
        self.direction_down = 0
        self.direction_left = 0
        self.direction_right = 0
        self.degree = -1  # 0: 少し, 1: たくさん
        self.syntaxes = []
        self.lemma_lst = []
        self.called_count = 0

        self.set_words()

        self.actionlib_client = actionlib.SimpleActionClient("/analyze_text/text", AnalyzeTextAction)
        self.actionlib_client.wait_for_server()

        rospy.Subscriber("/input_gcl_text", String, self.gcl_cb)

    def set_words(self):
        self.clear_direction = ["上", "下", "左", "右"]
        self.request = ["ほしい", "方が"]
        self.objects = ["飾り", "風船"]
        self.indicator_objects = ["これ", "それ", "あれ"]
        self.degree_words = ["もっと", "たくさん", "少し", "ちょっと"]
        self.indicator_direction = ["そっち", "こっち", "あっち", "ここ", "あそこ"]
        self.indicator_here = ["そこ"]
        self.excess_and_denial = ["過ぎ", "逆", "違う", "戻し"]
        self.agree = ["うん", "はい", "良い"]
        self.action_verb = ["置い", "する", "動かす", "動かし"]

        self.triggers = self.objects + self.indicator_objects + self.degree_words + self.indicator_direction + self.indicator_here

    def convert_sentence(self, sentence):
        sentence = sentence.replace("よい", "良い")
        sentence = sentence.replace("いい", "良い")
        sentence = sentence.replace("ほう", "方")
        sentence = sentence.replace("すぎ", "過ぎ")
        sentence = sentence.replace("ちがう", "違う")
        sentence = sentence.replace("もどし", "戻し")
        return sentence

    def get_parsing_result(self, input_sentence):
        pub_action_msg = AnalyzeTextGoal()
        pub_action_msg.text = input_sentence
        self.actionlib_client.send_goal(pub_action_msg)
        self.actionlib_client.wait_for_result()
        result = self.actionlib_client.get_result()
        return result

    def gcl_cb(self, msg):
        if msg.data == "":  # for debug
            self.called_count = 0
        else:
            input_sentence = self.convert_sentence(msg.data)
            result = self.get_parsing_result(input_sentence)
            self.syntaxes = result.syntaxes
            self.lemma_lst = []
            for syntax in self.syntaxes:
                self.lemma_lst.append(syntax.lemma)

            if self.called_count == 0:
                response = self.first_call(input_sentence)
            else:
                response = self.interactive_call(input_sentence)
            self.get_degree(input_sentence)

            print(input_sentence)
            print("-> " + response)
            if self.degree == 0:
                print("程度：少し")
            elif self.degree == 1:
                print("程度：たくさん")
            print("")
            self.called_count += 1

    def first_call(self, input_sentence):
        # 上下左右
        response = self.get_clear_direction(input_sentence)
        if response:
            return response
        # トリガーワード
        trigger_flag = False
        for trigger in self.triggers:
            if trigger in input_sentence:
                trigger_flag = True
        if trigger_flag:
            for request in self.request:
                return "上下左右どちらに動かしたら良いですか？"
        # お願い語
        for i, request in enumerate(self.request):
            if request in input_sentence:
                if request == "方が":
                    request = "方"
                if self.is_head_index_instruct(request):
                    return "上下左右どちらに動かしたら良いですか？"

        return "指示モードに入らない"

    def interactive_call(self, input_sentence):
        # 上下左右
        response = self.get_clear_direction(input_sentence)
        if response:
            return response
        # 曖昧方向指示
        for indicator_direction in self.indicator_direction:
            if indicator_direction in input_sentence:
                return "ごめんなさい、代わりに置いてもらえますか？"
        # 否定
        if self.is_neg(input_sentence):
            if self.direction_left == 1:
                self.direction_right = 1
                self.direction_left = 0
            if self.direction_right == 1:
                self.direction_left = 1
                self.direction_right = 0
            if self.direction_up == 1:
                self.direction_down = 1
                self.direction_up = 0
            if self.direction_down == 1:
                self.direction_up = 1
                self.direction_down = 0
            move_direction = self.get_move_direction()
            if move_direction:
                return "風船を" + move_direction + "に動かします"
        # 程度
        self.get_degree(input_sentence)
        if self.degree > -1:
            move_direction = self.get_move_direction()
            if move_direction:
                return "風船を" + move_direction + "に動かします"
        # 肯定
        for agree in self.agree:
            if agree in input_sentence:
                return "対話指示モード終了"

        return "上下左右どっちに動かしたら良いですか？"

    def get_clear_direction(self, input_sentence):

        def store_direction(direction_word):
            if "左" in direction_word:
                self.direction_left = 1
            if "右" in direction_word:
                self.direction_right = 1
            if "上" in direction_word:
                self.direction_up = 1
            if "下" in direction_word:
                self.direction_down = 1
        # 上下左右
        direction_flag = False
        for direction in self.clear_direction:
            if direction in input_sentence:
                direction_flag = True
                if not self.is_head_index_neg(direction):
                    if direction == "左":
                        self.direction_left = 1
                    elif direction == "右":
                        self.direction_right = 1
                    elif direction == "上":
                        self.direction_up = 1
                    elif direction == "下":
                        self.direction_down = 1
        if not direction_flag:
            return ""
        move_direction = self.get_move_direction()
        if move_direction:
            return "風船を" + move_direction + "に動かします"
        if not self.is_head_index_neg(move_direction):
            store_direction(move_direction)
            return "風船を" + move_direction + "に動かします"
        return ""

    def is_head_index_instruct(self, word):
        if not word in self.lemma_lst:
            return False
        word_pos = self.lemma_lst.index(word)
        for syntax in self.syntaxes:
            if syntax.dependency_edge == word_pos:
                if syntax.lemma in self.action_verb:
                    return True
        return False

    def is_head_index_neg(self, word):
        if not word in self.lemma_lst:
            return True
        word_pos = self.lemma_lst.index(word)
        for syntax in self.syntaxes:
            if syntax.dependency_edge == word_pos:
                if syntax.parse_label == 25:
                    return True
        return False

    def is_neg(self, input_sentence):
        for syntax in self.syntaxes:
            if syntax.parse_label == 25:
                return True
        for neg in self.excess_and_denial:
            if neg in input_sentence:
                return True
        return False
    
    def get_move_direction(self):
        move_direction = ""
        if self.direction_right == 1:
            move_direction += "右"
        if self.direction_left == 1:
            move_direction += "左"
        if self.direction_up == 1:
            move_direction += "上"
        if self.direction_down == 1:
            move_direction += "下"
        return move_direction

    def get_degree(self, input_sentence):
        self.degree = -1
        for degree in self.degree_words:
            if degree in input_sentence:
                if degree in ["もっと", "たくさん"]:
                    self.degree = 1
                elif degree in ["少し", "ちょっと"]:
                    self.degree = 0

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
