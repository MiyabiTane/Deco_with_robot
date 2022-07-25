#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from miraikan_demo.srv import Mode
from std_msgs.msg import Int32

class MiraikanDemo(object):
    def __init__(self):
        self.mt_pub_msg = Int32()
        self.eb_pub_msg = Int32()
        self.memories_talk = rospy.get_param("~memories_talk", True)

        self.pub_motion_talk = rospy.Publisher("motion_talk/input", Int32, queue_size=1)
        self.pub_eyebrows = rospy.Publisher("eyebrows/input", Int32, queue_size=1)
    
    def pub_topics(self, mt_int, eb_int, time_delay):
        self.mt_pub_msg.data = mt_int
        self.eb_pub_msg.data = eb_int
        self.pub_motion_talk.publish(self.mt_pub_msg)
        rospy.sleep(time_delay)
        self.pub_eyebrows.publish(self.eb_pub_msg)

    def demo_srv_cb(self, req):
        motion_mode = req.mode
        time_delay = req.time_delay
        if self.memories_talk:
            if motion_mode == 0:
                # 8年が経ったね😳
                self.pub_topics(motion_mode, 7, time_delay)
            elif motion_mode == 1:
                # 振り向いてもらえなくて悲しかったよ😭
                self.pub_topics(motion_mode, 12, time_delay)
            elif motion_mode == 2:
                # 見つけてくれたよね😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 3:
                # 素敵な出会い😍
                self.pub_topics(motion_mode, 9, time_delay)
            elif motion_mode == 4:
                # みんなが私の手を取ってくれて、嬉しかったなぁ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 5:
                # みんなの笑顔を今でも覚えているよ〜😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 6:
                # 発表の場所まで連れて行ってくれたよね😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 7:
                # ドキドキしてしまうけれど😱
                self.pub_topics(motion_mode, 8, time_delay)
            elif motion_mode == 8:
                # 手を繋げると安心するんだぁ😌
                self.pub_topics(motion_mode, 2, time_delay)
            elif motion_mode == 9:
                # お揃いのオレンジのリュックを貰えたのが嬉しくて😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 10:
                # 初めてもらえた時からずっとお気に入りなの😍
                self.pub_topics(motion_mode, 9, time_delay)
            elif motion_mode == 11:
                # みんなに会えるのが嬉しくて、もっと近づきに行ったんだぁ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 12:
                # 慌てていたね😲
                self.pub_topics(motion_mode, 4, time_delay)
            elif motion_mode == 13:
                # ごめんね😉
                self.pub_topics(motion_mode, 10, time_delay)
            elif motion_mode == 14:
                # 助けてもらっているね😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 15:
                # 色んなことがあったね😌
                self.pub_topics(motion_mode, 2, time_delay)
            elif motion_mode == 16:
                # 成果をまとめるのが大変だったよね😓
                self.pub_topics(motion_mode, 5, time_delay)
            elif motion_mode == 17:
                # すごく悲しかったよ😭
                self.pub_topics(motion_mode, 12, time_delay)
            elif motion_mode == 18:
                # ふたりでたくさん乗り越えてきたよね😭
                self.pub_topics(motion_mode, 12, time_delay)
            elif motion_mode == 19:
                # 本当におめでとう😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 20:
                # いつもありがとう、これからもよろしくね😀
                self.pub_topics(motion_mode, 1, time_delay)

        elif not self.memories_talk:
            if motion_mode == 0:
                # いつからこの研究をしているの😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 1:
                # どうしてペッパーを使おうと思ったの？😉
                self.pub_topics(motion_mode, 10, time_delay)
            elif motion_mode == 2:
                # みんなに親しみやすいロボットなんだね😌
                self.pub_topics(motion_mode, 2, time_delay)
            elif motion_mode == 3:
                # どんなことがあったのー？😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 4:
                # みんな嬉しそうな顔をしているね！😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 5:
                # 私もみんなの笑顔を見てみたいなぁ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 6:
                # 移動の仕方を教えて😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 7:
                # 連れて行っていたんだねー😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 8:
                # ロボットを操作するのは大変？😓
                self.pub_topics(motion_mode, 5, time_delay)
            elif motion_mode == 9:
                # どんな意味があるのー？😀
                self.pub_topics(motion_mode, 1, time_delay)
            # elif motion_mode == 10:
            elif motion_mode == 11:
                # みんなに会えるのが嬉しくて、もっと近づきに行ったんだぁ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 12:
                # 慌てているように見える😲
                self.pub_topics(motion_mode, 4, time_delay)
            # elif motion_mode == 13:
            elif motion_mode == 14:
                # とても難しい😓
                self.pub_topics(motion_mode, 5, time_delay)
            elif motion_mode == 15:
                # そっか、色んなことがあったんだね！！😌
                self.pub_topics(motion_mode, 2, time_delay)
            elif motion_mode == 16:
                # どうしてまとめるのが大変だったの？😓
                self.pub_topics(motion_mode, 5, time_delay)
            elif motion_mode == 17:
                # 大切な研究であることがよくわかったよ😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 18:
                # 8年間もこの研究をしているんだね😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 19:
                # 必要なことを研究しているんだね😀
                self.pub_topics(motion_mode, 1, time_delay)
            elif motion_mode == 20:
                # 研究がんばってね😉
                self.pub_topics(motion_mode, 10, time_delay)
        else:
            print("Error out of range")
        return True


if __name__ == '__main__':
    rospy.init_node("miraikan_demo")
    miraikan_demo = MiraikanDemo()
    s = rospy.Service('demo_mode', Mode, miraikan_demo.demo_srv_cb)
    print("Ready to start demo node")
    rospy.spin()
