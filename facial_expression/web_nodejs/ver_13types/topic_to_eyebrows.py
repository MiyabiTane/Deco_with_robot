#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import subprocess
from std_msgs.msg import Int32

class TopicToEyebrows(object):
    def __init__(self):
        rospy.Subscriber("/eyebrows/input_type", Int32, self.eyebrows_cb)
    
    def eyebrows_cb(self, msg):
        """
        0. Normal
        1. Happy😀
        2. Relieved😌
        3. Smirking😏
        4. Astonished😲
        5. Unpleasant😓
        6. Angry😠
        7. Flushed😳
        8. Fearful😱
        9. Love😍
        10. Squinting😝
        11. Boring😪
        12. Cry😭
        """
        degrees = [0, 20, 90, 70, 50, 30, 20, 60, 70, 120, 70, 130, 50]
        subprocess.call(["curl", "-X", "POST", "--data-urlencode", "mode=" + str(msg.data),
                         "--data-urlencode", "degree=" + str(degrees[msg.data]), "http://localhost:3000/api/info"])

if __name__ == '__main__':
    rospy.init_node("topic_to_eyebrows")
    topic_to_eyebrows = TopicToEyebrows()
    rospy.spin()
