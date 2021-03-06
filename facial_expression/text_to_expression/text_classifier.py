import json
# import emoji
# import numpy as np
import subprocess
import regex
import re

class MakeExpressionJson:
    def __init__(self):
        self.res_happy = []
        self.res_relieved = []
        self.res_smirking = []
        self.res_astonished = []
        self.res_crying = []
        self.res_enraged = []
        self.res_flushed = []
        self.res_fearful = []
        self.res_love = []
        self.res_squinting = []
        self.res_sleepy = []
        self.res_anxious = []

    def in_emoji(self, words):
        # https://note.nkmk.me/python-re-regex-character-type/
        p = regex.compile(r'\p{Emoji=Yes}+')
        emojis = ''.join(p.findall(words))
        for emoji_ in emojis:
            if emoji_ in "๐๐๐๐๐๐๐คฃ๐๐๐๐๐๐๐ฅฐ๐๐คฉ๐๐๐๐๐๐๐๐คช๐๐ค๐ค๐คญ๐คซ๐ค๐ค๐คจ๐๐๐ถ๐๐๐๐ฌ๐คฅ๐๐๐ช๐คค๐ด๐ท๐ค๐ค๐คข๐คฎ๐คง๐ฅต๐ฅถ๐ฅด๐ต๐คฏ๐ค ๐ฅณ๐๐ค๐ง๐๐๐๐ฎ๐ฏ๐ฒ๐ณ๐ฅบ๐ฆ๐จ๐ฐ๐ฅ๐ข๐ญ๐ฑ๐๐ฃ๐๐๐ฉ๐ซ๐ค๐ก๐ ๐คฌ":
                return True
        p = regex.compile(r'\p{Emoji_Presentation=Yes}+')
        emojis = ''.join(p.findall(words))
        for emoji_ in emojis:
            if emoji_ in "๐๐๐๐๐๐คฃ๐๐๐๐๐๐๐ฅฐ๐๐คฉ๐๐๐๐๐๐๐๐คช๐๐ค๐ค๐คญ๐คซ๐ค๐ค๐คจ๐๐๐ถ๐๐๐๐ฌ๐คฅ๐๐๐ช๐คค๐ด๐ท๐ค๐ค๐คข๐คฎ๐คง๐ฅต๐ฅถ๐ฅด๐ต๐คฏ๐ค ๐ฅณ๐๐ค๐ง๐๐๐๐ฎ๐ฏ๐ฒ๐ณ๐ฅบ๐ฆ๐จ๐ฐ๐ฅ๐ข๐ญ๐ฑ๐๐ฃ๐๐๐ฉ๐ซ๐ค๐ก๐ ๐คฌ":
                return True
        return False

    def make_emoji_json(self, y_ids):
        dic_ = {}
        id_ = 0
        for y_id in y_ids:
            json_name = "comments/" + y_id + ".json"
            decoder = json.JSONDecoder()
            with open(json_name, 'r') as f:
                line = f.readline()
                while line:
                    content = decoder.raw_decode(line)[0]["text"]
                    if self.in_emoji(content):
                        dic_[id_] = content
                        id_ += 1
                    line = f.readline()
        with open('emoji_sentence.json', 'w') as f:
            json.dump(dic_, f, indent=4, ensure_ascii=False)

    def arrange_sentence(self, sentence):
        # ๆๅญใฎ็ฝฎใๆใ
        replace_team = ["ใใฃใใทใฃใผใบ", "Fischer's", "ใตใใฃใใใผใ", "ใตใใใใผใ", "ใตใใฃใใใผใ", "ใใฃใทใฃใผใบ", "ใใคใใทใฃใผใบ", "ใใใทใฃใผใบ", "ใฒใใฃใใใผใ", "ใใฃใฃใทใฅใผใบ"]
        replace_obj = ["ใญใณใซใธใฅใผ", "ใกใใญใใ", "ใใใใใใผ", "ใใชใผใดใผใซใ", "่ฃธใฎๅฟ"]
        for team in replace_team:
            sentence = sentence.replace(team, "ๅฝผใ")
        for obj in replace_obj:
            sentence = sentence.replace(obj, "ใใ")
        # @hogeใๅใ้คใ
        if '@' in sentence:
            where_at = sentence.find('@')
            cur_c = "@"
            replace_words = ""
            while (not cur_c in [" ", "ใ"]):
                cur_c = sentence[where_at]
                replace_words += cur_c
                where_at += 1
                if where_at == len(sentence):
                    break
            sentence = sentence.replace(replace_words, "")
        return sentence

    def extract_sentence(self, emoji, content):
        where_emoji = content.find(emoji)
        content = content[:where_emoji]
        content = self.arrange_sentence(content)
        where_n = content.rfind('\n')
        if where_n <= 0:
            where_n = content.rfind(' ')
        if where_n > 0:
            return content[where_n + 1:], where_emoji
        return content, where_emoji

    def make_json(self, res_lst, intent_name):
        intent_dic =  {"id": "", "name": intent_name, "auto": True, "condition": "",
                        "conditionalFollowupEvents": [], "conditionalResponses": [],
                        "context": [], "contexts": [], "endInteraction": False, "events": [],
                        "fallbackIntent": False, "liveAgentHandoff": False, "parentId": None,
                        "followUpIntents": [], "priority": 500000, "responses": [],
                        "rootParentId": None, "templates": [], "userSays": [],
                        "webhookForSlotFilling": False, "webhookUsed": False}
        responses_dic = {"action": "", "affectedContexts": [], "parameters": [], "defaultResponsePlatforms": {},
                         "messages": [{"type": "message", "condition": "", "speech": []}],
                         "resetContexts": False}
        intent_dic["responses"] = responses_dic
        for sentence in res_lst:
            usersays_dic = {"isTemplate": False, "data": [], "count": 0, "id": "", "updated": None}
            data_dic = {}
            data_dic["text"] = sentence
            data_dic["userDefined"] = False
            usersays_dic["data"].append(data_dic)
            intent_dic["userSays"].append(usersays_dic)

        with open("results/" + intent_name + '.json', 'w') as f:
            json.dump(intent_dic, f, indent=4, ensure_ascii=False)
    
    def is_ignore(self, sentence):
        ignore_words = re.findall('ใ|ๅข|ใใ|ใทใซใฏ|ใใตใค|ใขใใญ|ใใผใ|ใบใใใ|ใณใใ|ใถใซใช|ใใใฟ|่ฃธ|ใใใซ|ใฏใญใผใ|ใใใผใฉ|๏พ๏พ๏พ๏พ|ใพใใ|@|ใดใผใซใ|ใใ|ใใ ใป|ใใจใ|ใใใใใช|ใใฑใฟใณ|ใ|ใใฃใใใ|ใฟใใผใผใฟ|๏พ', sentence)
        # ใดใผใซใ๏ผใใชใผใดใผใซใใ่ฃธ๏ผ่ฃธใฎๅฟใๆฒๅใ"ใใ"ใซ็ฝฎใๆใใใใใงๆฎใฃใฆใใใใฎใๆถใใฆใใ
        if len(ignore_words) > 0:
            return True
        p = re.compile('[a-z]+')
        if len(''.join(p.findall(sentence))) > 0:
            return True
        p = re.compile('[A-Z]+')
        if len(''.join(p.findall(sentence))) > 0:
            return True
        if "(" in sentence:  # ้กๆๅญ
            return True
        if ")" in sentence:
            return True
        # if ":" in sentence:
        #     return True
        p = regex.compile(r'\p{Script=Han}+')
        if len(''.join(p.findall(sentence))) == len(sentence):  # ๅจ้จๆผขๅญ๏ผไธญๅฝ่ช๏ผ
            return True
        p = re.compile('[\u30A1-\u30FF]+')
        if len(''.join(p.findall(sentence))) > 4:  # ใซใฟใซใ่จ่
            return True
        p = re.compile('[\u3041-\u309F]+')
        if len(''.join(p.findall(sentence))) == 0:  # ใฒใใใชใชใ
            return True
        p = regex.compile(r'\p{Emoji=Yes}+')  # ้กๆๅญไปฅๅคใฎ็ตตๆๅญ
        if len(''.join(p.findall(sentence))) > 0:
            return True
        if "แแฎ" in sentence:
            return True
        if "ุชุฌูู"  in sentence:
            return True
        if "์" in sentence:
            return True
        if "๊" in sentence:
            return True
        return False

    def extract_and_append(self, emojis_str):
        debug = False
        # if "hoge" in emojis_str:
        #     debug = True
        p = regex.compile(r'\p{Emoji=Yes}+')
        emojis = ''.join(p.findall(emojis_str))
        # print("== ", emojis_str)
        for emoji_ in emojis:
            if not emoji_ in "๐๐๐๐๐๐๐๐๐ฎ๐ฏ๐ฒ๐ข๐ญ๐ก๐ ๐คฌ๐ค๐ณ๐ฅบ๐ฑ๐ซ๐จ๐ฅฐ๐๐๐คฃ๐๐คฉ๐๐๐คช๐๐๐ช๐๐๐ด๐๐ฆ๐๐ฃ๐๐๐ฅ๐๐๐ฐ":
                if emoji_.isdigit():
                    continue
                else:
                    where_emoji = emojis_str.find(emoji_)
                    emojis_str = emojis_str[:where_emoji] + emojis_str[where_emoji+1:]
                    continue
            else:  # ็ตตๆๅญใ้ฃ็ถใงไฝฟใใใฆใใฆๅ้ ญใซ้ก็ตตๆๅญใๆฅใฆใใพใๆ
                where_emoji = emojis_str.find(emoji_)
                if where_emoji == 0:
                    emojis_str = emojis_str[1:]
                    continue
            if debug:
                print(emojis_str)
            sentence, where_n = self.extract_sentence(emoji_, emojis_str)
            if self.is_ignore(sentence):
                continue
            sentence = sentence.strip()
            if len(sentence) > 2:
                # print(emoji_, sentence, where_n)
                if emoji_ in "๐๐๐๐๐๐":  # happy
                    self.res_happy.append(sentence)
                if emoji_ in "๐":  # relieved
                    self.res_relieved.append(sentence)
                if emoji_ in "๐":  # smirking
                    self.res_smirking.append(sentence) 
                if emoji_ in "๐ฎ๐ฏ๐ฒ":  # astonished
                    self.res_astonished.append(sentence)
                if emoji_ in "๐ข๐ญ":  # Cry๏ผ๐๏ผ
                    self.res_crying.append(sentence)
                if emoji_ in "๐ก๐ ๐คฌ":  # angry
                    self.res_enraged.append(sentence)
                if emoji_ in "๐ณ":  # flushed
                    self.res_flushed.append(sentence)
                if emoji_ in "๐ฑ๐จ":  # fearful
                    self.res_fearful.append(sentence)
                if emoji_ in "๐ฅฐ๐๐":  # love
                    self.res_love.append(sentence)
                if emoji_ in "๐คฉ๐๐๐คช๐๐":  # squinting๏ผ๐คฃ๏ผ
                    self.res_squinting.append(sentence)
                if emoji_ in "๐ช๐๐ด":  # boring (sleepy)
                    self.res_sleepy.append(sentence)
                if emoji_ in "๐๐๐๐ฅ๐๐ฐ๐ฆ๐":  # unpleasant๏ผ๐ใฏใใใจ้้ใใไบบใๅคใใใ ใใ้คใใ๏ผ
                    self.res_anxious.append(sentence)
            emojis_str = emojis_str[where_n + 1 :]

    def make_intent_json(self, json_name):
        json_open = open(json_name, 'r')
        json_load = json.load(json_open)
        for i in range(len(json_load)):
            content = json_load[str(i)]
            self.extract_and_append(content)
        self.make_json(self.res_happy, "Happy")
        self.make_json(self.res_relieved, "Relived")
        self.make_json(self.res_smirking, "Smirking")
        self.make_json(self.res_astonished, "Astonished")
        self.make_json(self.res_crying, "Cry")
        self.make_json(self.res_enraged, "Angry")
        self.make_json(self.res_flushed, "Flushed")
        self.make_json(self.res_fearful, "Fearful")
        self.make_json(self.res_love, "Love")
        self.make_json(self.res_squinting, "Squinting")
        self.make_json(self.res_sleepy, "Boring")
        self.make_json(self.res_anxious, "Unpleasant")

YOUTUBE_IDS = ["0xI4J9CwMuY", "Na_WJPK26Oc", "28jAR_LDNJE", "uxk_qap7pwA", "VadBq-_234g", "0xSiBpUdW4E", "yOAwvRmVIyo", "pfGI91CFtRg", "9qRCARM_LfE", "ARwVe1MYAUA"]
makeExpressionJson = MakeExpressionJson()

# for y_id in YOUTUBE_IDS:
#     subprocess.call(["pipenv", "run", "youtube-comment-downloader", "--youtubeid", y_id, "--output",
#                      "comments/" + y_id + ".json", "--language", "ja", "--limit", "50000"]) # "--sort", "0",])
# makeExpressionJson.make_emoji_json(YOUTUBE_IDS)
# print("Make Json Done")

makeExpressionJson.make_intent_json("emoji_sentence.json")
