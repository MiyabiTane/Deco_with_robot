#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess
import argparse
import time
from std_msgs.msg import String

class OpenBrowser(object):
    def __init__(self, mode):
        rospy.Subscriber("/facial_expression/ip_info", String, self.ip_cb)
        rospy.Subscriber("/facial_expression/restart", String, self.restart_cb)
        self.mode = mode
    
    def ip_cb(self, msg):
        print("OPEN ", "http://" + msg.data + ":3000/" + self.mode)
        subprocess.call(["chromium-browser", "http://" + msg.data + ":3000/" + self.mode, "--kiosk"])

    def restart_cb(self, msg):
        print("RESTART_SYSTEM")
        if self.mode == "lbrow":
            subprocess.call(["systemctl", "--user", "restart", "open-browser-l.service"])
        elif self.mode == "rbrow":
            subprocess.call(["systemctl", "--user", "restart", "open-browser-r.service"])


if __name__ == '__main__':
    rospy.init_node("open_browser")
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode")
    args = parser.parse_args()
    OpenBrowser(args.mode)
    rospy.spin()
