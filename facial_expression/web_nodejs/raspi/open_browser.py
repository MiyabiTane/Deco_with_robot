#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess
import argparse
from std_msgs.msg import String

class OpenBrowser(object):
    def __init__(self, mode):
        rospy.Subscriber("/facial_expression/ip_info", String, self.ip_cb)
        self.mode = mode
    
    def ip_cb(self, msg):
        subprocess.call(["chromium-browser", "http://" + msg.data + "/" + mode, "--kiosk"])

if __name__ == '__main__':
    rospy.init_node("open_browser")
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode")
    args = parser.parse_args()
    OpenBrowser(args.mode)
    rospy.spin()
