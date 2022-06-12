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
            if emoji_ in "😀😃😄😁😆😅🤣😂🙂🙃😉😊😇🥰😍🤩😘😗😚😙😋😛😜🤪😝🤑🤗🤭🤫🤔🤐🤨😐😑😶😏😒🙄😬🤥😌😔😪🤤😴😷🤒🤕🤢🤮🤧🥵🥶🥴😵🤯🤠🥳😎🤓🧐😕😟🙁😮😯😲😳🥺😦😨😰😥😢😭😱😖😣😞😓😩😫😤😡😠🤬":
                return True
        p = regex.compile(r'\p{Emoji_Presentation=Yes}+')
        emojis = ''.join(p.findall(words))
        for emoji_ in emojis:
            if emoji_ in "😃😄😁😆😅🤣😂🙂🙃😉😊😇🥰😍🤩😘😗😚😙😋😛😜🤪😝🤑🤗🤭🤫🤔🤐🤨😐😑😶😏😒🙄😬🤥😌😔😪🤤😴😷🤒🤕🤢🤮🤧🥵🥶🥴😵🤯🤠🥳😎🤓🧐😕😟🙁😮😯😲😳🥺😦😨😰😥😢😭😱😖😣😞😓😩😫😤😡😠🤬":
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
        # 文字の置き換え
        replace_team = ["フィッシャーズ", "Fischer's", "ふぃっしゃーず", "ふぃしゃーず", "ふぁっしゃーず", "フィシャーズ", "フイッシャーズ", "フッシャーズ", "ひぃっしゃーず", "フィっシューズ"]
        replace_obj = ["キンカジュー", "メダロット", "きんかじゅー", "マリーゴールド", "裸の心"]
        for team in replace_team:
            sentence = sentence.replace(team, "彼ら")
        for obj in replace_obj:
            sentence = sentence.replace(obj, "それ")
        # @hogeを取り除く
        if '@' in sentence:
            where_at = sentence.find('@')
            cur_c = "@"
            replace_words = ""
            while (not cur_c in [" ", "　"]):
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
        ignore_words = re.findall('『|垢|さん|シルク|マサイ|モトキ|ダーマ|ぺけたん|ンダホ|ザカオ|あいみ|裸|ハダカ|クロード|くろーど|ﾝﾀﾞﾎ|まさい|@|ゴールド|くん|んだほ|もとき|あかさたな|ペケタン|「|もっきゅん|みぃーーみ|ﾝ', sentence)
        # ゴールド：マリーゴールド、裸：裸の心　曲名を"それ"に置き換えたうえで残っているものを消している
        if len(ignore_words) > 0:
            return True
        p = re.compile('[a-z]+')
        if len(''.join(p.findall(sentence))) > 0:
            return True
        p = re.compile('[A-Z]+')
        if len(''.join(p.findall(sentence))) > 0:
            return True
        if "(" in sentence:  # 顔文字
            return True
        if ")" in sentence:
            return True
        # if ":" in sentence:
        #     return True
        p = regex.compile(r'\p{Script=Han}+')
        if len(''.join(p.findall(sentence))) == len(sentence):  # 全部漢字（中国語）
            return True
        p = re.compile('[\u30A1-\u30FF]+')
        if len(''.join(p.findall(sentence))) > 4:  # カタカナ言葉
            return True
        p = re.compile('[\u3041-\u309F]+')
        if len(''.join(p.findall(sentence))) == 0:  # ひらがななし
            return True
        p = regex.compile(r'\p{Emoji=Yes}+')  # 顔文字以外の絵文字
        if len(''.join(p.findall(sentence))) > 0:
            return True
        if "ဒီ" in sentence:
            return True
        if "تجنن"  in sentence:
            return True
        if "아" in sentence:
            return True
        if "ꉂ" in sentence:
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
            if not emoji_ in "😀😃😄😁😆😊😌😏😮😯😲😢😭😡😠🤬😤😳🥺😱😫😨🥰😍😘🤣😂🤩😉😜🤪😝🙄😪😔😒😴🙁😦😖😣😞😓😥😕😟😰":
                if emoji_.isdigit():
                    continue
                else:
                    where_emoji = emojis_str.find(emoji_)
                    emojis_str = emojis_str[:where_emoji] + emojis_str[where_emoji+1:]
                    continue
            else:  # 絵文字が連続で使われていて先頭に顔絵文字が来てしまう時
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
                if emoji_ in "😀😃😄😁😆😊":  # happy
                    self.res_happy.append(sentence)
                if emoji_ in "😌":  # relieved
                    self.res_relieved.append(sentence)
                if emoji_ in "😏":  # smirking
                    self.res_smirking.append(sentence) 
                if emoji_ in "😮😯😲":  # astonished
                    self.res_astonished.append(sentence)
                if emoji_ in "😢😭":  # Cry（😂）
                    self.res_crying.append(sentence)
                if emoji_ in "😡😠🤬":  # angry
                    self.res_enraged.append(sentence)
                if emoji_ in "😳":  # flushed
                    self.res_flushed.append(sentence)
                if emoji_ in "😱😨":  # fearful
                    self.res_fearful.append(sentence)
                if emoji_ in "🥰😍😘":  # love
                    self.res_love.append(sentence)
                if emoji_ in "🤩😉😜🤪😝🙄":  # squinting（🤣）
                    self.res_squinting.append(sentence)
                if emoji_ in "😪😒😴":  # boring (sleepy)
                    self.res_sleepy.append(sentence)
                if emoji_ in "😖😞😓😥😟😰😦🙁":  # unpleasant（😔はホッと間違える人が多そうだから除いた）
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
