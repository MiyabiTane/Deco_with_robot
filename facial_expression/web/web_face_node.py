#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import time
import subprocess
import threading
from std_msgs.msg import Float64MultiArray

IMG_PATH_LBROW = "static/lbrow.jpg"
IMG_PATH_RBROW = "static/rbrow.jpg"
IMG_PATH_MOUTH = "static/mouth.jpg"

SIZE_X = 640
SIZE_Y = 480
BLOW_LEN = 400
BLOW_WID = 100

class FacialExpressNode:
    def __init__(self):
        self.score = 0
        self.magnitude = 0
        rospy.Subscriber('input', Float64MultiArray, self.face_exp_cb)

        thread_1 = threading.Thread(target=self.run_server_1)
        thread_2 = threading.Thread(target=self.run_server_2)
        thread_3 = threading.Thread(target=self.run_server_3)
        thread_1.start()
        thread_2.start()
        thread_3.start()
    
    def run_server_1(self):
        subprocess.call(["pipenv", "run", "python", "app_1.py", "--debugger", "--reload"])
    
    def run_server_2(self):
        subprocess.call(["pipenv", "run", "python", "app_2.py"])
    
    def run_server_3(self):
        subprocess.call(["pipenv", "run", "python", "app_3.py"])

    def face_exp_cb(self, msg):
        if len(msg.data) > 0:
            self.score = msg.data[0]  # -1: ネガティブ感情, 1: ポジティブ感情
            if len(msg.data) > 1:
                self.magnitude = msg.data[1]  # 感情の強さ
        print("score, magnitude: ", self.score, self.magnitude)
        self.update_lbrow(self.score * 20)
        self.update_rbrow(self.score * 20)
        self.update_mouth(self.score * 20)

    def update_lbrow(self, deg):
        img = np.full((SIZE_Y, SIZE_X, 3), 225, dtype=np.uint8)
        rad = deg * np.pi / 180
        pt1 = (120, 180)
        pt2 = (int(pt1[0] + BLOW_WID * np.sin(rad)), int(pt1[1] + BLOW_WID * np.cos(rad)))
        pt3 = (int(pt2[0] + BLOW_LEN * np.cos(rad)), int(pt2[1] - BLOW_LEN * np.sin(rad)))
        pt4 = (int(pt3[0] - BLOW_WID * np.sin(rad)), int(pt3[1] - BLOW_WID * np.cos(rad)))
        pts = np.array((pt1, pt2, pt3, pt4))
        cv2.fillPoly(img, [pts], (0, 0, 0))
        # 眉端
        pt5 = (int(pt3[0] + 20 * np.cos(rad)), int(pt3[1] - 20 * np.sin(rad)))
        pts = np.array((pt3, pt4, pt5))
        cv2.fillPoly(img, [pts], (0, 0, 0))
        cv2.imwrite(IMG_PATH_LBROW, img)
        print("UPDATE: ", IMG_PATH_LBROW)

    def update_rbrow(self, deg):
        img = np.full((SIZE_Y, SIZE_X, 3), 225, dtype=np.uint8)
        rad = deg * np.pi / 180
        pt1 = (520, 180)
        pt2 = (int(pt1[0] - BLOW_WID * np.sin(rad)), int(pt1[1] + BLOW_WID * np.cos(rad)))
        pt3 = (int(pt2[0] - BLOW_LEN * np.cos(rad)), int(pt2[1] - BLOW_LEN * np.sin(rad)))
        pt4 = (int(pt3[0] + BLOW_WID * np.sin(rad)), int(pt3[1] - BLOW_WID * np.cos(rad)))
        pts = np.array((pt1, pt2, pt3, pt4))
        cv2.fillPoly(img, [pts], (0, 0, 0))
        # 眉端
        pt5 = (int(pt3[0] - 20 * np.cos(rad)), int(pt3[1] - 20 * np.sin(rad)))
        pts = np.array((pt3, pt4, pt5))
        cv2.fillPoly(img, [pts], (0, 0, 0))
        cv2.imwrite(IMG_PATH_RBROW, img)
        print("UPDATE: ", IMG_PATH_RBROW)

    def update_mouth(self, deg):
        img = np.full((SIZE_Y, SIZE_X, 3), 225, dtype=np.uint8)
        rad = deg * np.pi / 180
        pt1 = (200, 240)
        pt2 = (440, 240)
        pt3 = (int(200 - 80 * np.cos(rad)), int(240 - 80 * np.sin(rad)))
        pt4 = (int(440 + 80 * np.cos(rad)), int(240 - 80 * np.sin(rad)))
        cv2.line(img, pt1, pt2, (0, 0, 0), thickness=60)
        cv2.line(img, pt1, pt3, (0, 0, 0), thickness=60)
        cv2.line(img, pt2, pt4, (0, 0, 0), thickness=60)
        cv2.imwrite(IMG_PATH_MOUTH, img)
        print("UPDATE: ", IMG_PATH_MOUTH)

if __name__ == '__main__':
    rospy.init_node("facial_exp")
    facial_express = FacialExpressNode()
    rospy.spin()