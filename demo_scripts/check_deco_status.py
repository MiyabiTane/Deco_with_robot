#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import glob

def get_match_pos(back_img, deco_img, thresh):
    res = cv2.matchTemplate(back_img, deco_img, cv2.TM_CCOEFF_NORMED)
    _min_val, max_val, _min_loc, max_loc = cv2.minMaxLoc(res)
    if max_val > thresh:
        H, W, _ = deco_img.shape
        lx, ly = max_loc
        cx = int(lx + W / 2.0)
        cy = int(ly + H / 2.0)
        return cx, cy
    return -1, -1

def check_status(manip_box_num):
    thresh = 0.9
    ga_img = cv2.imread("img/ga_output_0.jpg")
    res_img = cv2.imread("img/result_0.jpg")
    deco_img = cv2.imread("img/input" + str(manip_box_num) + ".jpg")
    ga_x, ga_y, res_x, res_y = -1, -1, -1, -1
    while -1 in [ga_x, ga_y, res_x, res_y]:
        print("thresh: ", thresh)
        ga_x, ga_y = get_match_pos(ga_img, deco_img, thresh)
        res_x, res_y = get_match_pos(res_img, deco_img, thresh)
        thresh -= 0.1
    # 今ロボットが飾り付けた画像を基準に他の飾りのずれを見る
    afin_matrix = np.float32([[1, 0, ga_x - res_x], [0, 1, ga_y - res_y]])
    res_img = cv2.warpAffine(res_img, afin_matrix, (640, 480))
    # cv2.imwrite("img/debug_view.jpg", res_img)

    decos_files = glob.glob("img/input*.jpg")
    for i in range(len(decos_files)):
        fi = "img/input" + str(i) + ".jpg"
        if fi == "img/input" + str(manip_box_num) + ".jpg":
            print("manip box")
        else:
            deco_img = cv2.imread(fi)
            ga_x, ga_y = get_match_pos(ga_img, deco_img, thresh)
            res_x, res_y = get_match_pos(res_img, deco_img, thresh)
            if -1 in [ga_x, ga_y, res_x, res_y]:
                diff_x, diff_y = -1, -1
            else:
                diff_x, diff_y = res_x - ga_x, res_y - ga_y
            print("diff_x, diff_y: ", diff_x, diff_y)

check_status(0)
