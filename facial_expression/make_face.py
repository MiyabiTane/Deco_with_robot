#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import time

SIZE_X = 640
SIZE_Y = 480
IMG_PATH = "./images/face.jpg"

BLOW_LEN = 70
BLOW_WID = 20

class FacialExpress:
    def __init__(self):
        self.img = self.make_img()

    def make_img(self):
        img = np.full((SIZE_Y, SIZE_X, 3), 225, dtype=np.uint8)
        input_img = cv2.imread(IMG_PATH)
        H, W, _ = input_img.shape
        magni = min(SIZE_Y // H, SIZE_X // W)
        input_img = cv2.resize(input_img, dsize=None, fx=magni, fy=magni)
        img[int((SIZE_Y - H*magni)/2): int((SIZE_Y - H*magni)/2) + H*magni,
            int((SIZE_X - W*magni)/2): int((SIZE_X - W*magni)/2) + W*magni] = input_img
        return img

    def show_img(self):
        for i in range(30):
            start_time = time.time()
            cv2.imshow("face", self.img)
            end_time = time.time()
            wait_time = max(200 - int((end_time - start_time) * 1000), 1)
            if cv2.waitKey(wait_time) >= 0:
                break

    def add_left_eyebrow(self, deg):
        rad = deg * np.pi / 180
        pt1 = (430, 140)
        pt2 = (int(pt1[0] + BLOW_WID * np.sin(rad)), int(pt1[1] + BLOW_WID * np.cos(rad)))
        pt3 = (int(pt2[0] + BLOW_LEN * np.cos(rad)), int(pt2[1] - BLOW_LEN * np.sin(rad)))
        pt4 = (int(pt3[0] - BLOW_WID * np.sin(rad)), int(pt3[1] - BLOW_WID * np.cos(rad)))
        pts = np.array((pt1, pt2, pt3, pt4))
        cv2.fillPoly(self.img, [pts], (0, 0, 0))
        # 眉端
        pt5 = (int(pt3[0] + 20 * np.cos(rad)), int(pt3[1] - 20 * np.sin(rad)))
        pts = np.array((pt3, pt4, pt5))
        cv2.fillPoly(self.img, [pts], (0, 0, 0))

    def add_right_eyebrow(self, deg):
        rad = deg * np.pi / 180
        pt1 = (200, 140)
        pt2 = (int(pt1[0] - BLOW_WID * np.sin(rad)), int(pt1[1] + BLOW_WID * np.cos(rad)))
        pt3 = (int(pt2[0] - BLOW_LEN * np.cos(rad)), int(pt2[1] - BLOW_LEN * np.sin(rad)))
        pt4 = (int(pt3[0] + BLOW_WID * np.sin(rad)), int(pt3[1] - BLOW_WID * np.cos(rad)))
        pts = np.array((pt1, pt2, pt3, pt4))
        cv2.fillPoly(self.img, [pts], (0, 0, 0))
        # 眉端
        pt5 = (int(pt3[0] - 20 * np.cos(rad)), int(pt3[1] - 20 * np.sin(rad)))
        pts = np.array((pt3, pt4, pt5))
        cv2.fillPoly(self.img, [pts], (0, 0, 0))
    
    def add_mouth(self, deg):
        rad = deg * np.pi / 180
        pt1 = (280, 330)
        pt2 = (370, 330)
        pt3 = (int(280 - 20 * np.cos(rad)), int(330 - 20 * np.sin(rad)))
        pt4 = (int(370 + 20 * np.cos(rad)), int(330 - 20 * np.sin(rad)))
        cv2.line(self.img, pt1, pt2, (0, 0, 0), thickness=10)
        cv2.line(self.img, pt1, pt3, (0, 0, 0), thickness=10)
        cv2.line(self.img, pt2, pt4, (0, 0, 0), thickness=10)

    def get_click_pos(self):
        positions = []
        def onMouse(event, x, y, flags, params):
            if event == cv2.EVENT_LBUTTONDOWN:
                positions.append([x, y])

        cv2.namedWindow('window', cv2.WINDOW_NORMAL)
        cv2.imshow('window', self.img)
        cv2.setMouseCallback('window', onMouse)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return positions

    def show_pos(self):
        print("左眉毛の位置をクリックして下さい")
        positions = self.get_click_pos()
        print(positions)
        print("右眉毛の位置をクリックして下さい")
        positions = self.get_click_pos()
        print(positions)
        print("口の位置をクリックして下さい")
        positions = self.get_click_pos()
        print(positions)

facial_express = FacialExpress()
facial_express.add_left_eyebrow(20)
facial_express.add_right_eyebrow(20)
facial_express.add_mouth(20)
facial_express.show_img()
# facial_express.show_pos()

"""
左眉毛の位置をクリックして下さい
[[99, 184], [190, 188], [180, 142], [99, 144]]
右眉毛の位置をクリックして下さい
[[434, 140], [438, 188], [521, 177], [518, 142]]
口の位置をクリックして下さい
[[194, 302], [201, 361], [436, 358], [434, 302]]
"""
