#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import glob
import copy

HIST_TH = 0.5

class CheckDecoStatus:
    def __init__(self):
        self.set_test_param()
        self.decorated_imgs = []
        self.input_imgs = []
        deco_files = glob.glob("img/1106/input*.jpg")
        for i in range(len(deco_files)):
            fi = "img/1106/input" + str(i) + ".jpg"
            deco_img = cv2.imread(fi)
            self.input_imgs.append(deco_img)

    def set_test_param(self):
        self.ideal_img = cv2.imread("img/1106/ga_output_1.jpg")
        self.result_img = cv2.imread("img/1106/deco_result.jpg")
        self.xs_lst = [[197.50213623046875, 298.4449157714844, 197.56639099121094, 298.4561767578125],
                        [278.831787109375, 375.3031005859375, 278.8482971191406, 375.2809143066406]]
        self.ys_lst = [[203.76055908203125, 203.76077270507812, 321.6370849609375, 321.6370849609375],
                        [270.8799133300781, 270.8799133300781, 363.64788818359375, 363.6478576660156]]
        # self.ga_calced_pos = [(283.0, 157.0), (389.0, 288.0), (320.0, 240.0)]
        self.ga_calced_pos = [(int(283 - 103/2.0), int(157 - 82/2.0)), (int(389 - 91/2.0), int(288 - 57/2.0)), (275, 245)]

    def reorder_point(self, xs, ys):
        points = []
        for x, y in zip(xs, ys):
            points.append((int(x), int(y)))
        points.sort(key=lambda x:x[0])
        left_lst = points[:2]
        right_lst = points[2:]
        left_lst.sort(key=lambda x:x[1])
        right_lst.sort(key=lambda x:x[1])
        lt, lb, rt, rb = left_lst[0], left_lst[1], right_lst[0], right_lst[1]
        return lt, lb, rt, rb
    
    def debug_draw_line(self, img, lt, lb, rt, rb, color=(255, 0, 0)):
        debug_img = copy.deepcopy(img)
        draw_point = [lt, rt, rb, lb]
        for i in range(len(draw_point)):
            n_i = 0 if i + 1 == len(draw_point) else i + 1
            cv2.line(debug_img, draw_point[i], draw_point[n_i], color, thickness=2, lineType=cv2.LINE_4)
        return debug_img

    def clip_found_decoration(self):
        debug_img = copy.deepcopy(self.result_img)
        for i in range(len(self.xs_lst)):
            lt, lb, rt, rb = self.reorder_point(self.xs_lst[i], self.ys_lst[i])
            debug_img = self.debug_draw_line(debug_img, lt, lb, rt, rb)
            decorated_img = self.result_img[min(lt[0], rt[0]): max(lb[0], rb[0]), min(lt[1], lb[1]): max(rt[1], rb[1])]
            cv2.imwrite("img/1106/decorated" + str(i) + ".jpg", decorated_img)
            self.decorated_imgs.append(decorated_img)
        cv2.imwrite("img/1106/decorated_box.jpg", debug_img)
    
    def get_similar_img_num(self, target_img, imgs):
        # use histgram of color
        ans_num = -1
        max_sim_point = HIST_TH
        h, w, _ = target_img.shape
        target_hist = cv2.calcHist([target_img], [0], None, [256], [0, 256])
        for i, img in enumerate(imgs):
            img = cv2.resize(img, (w, h))
            compare_hist = cv2.calcHist([img], [0], None, [256], [0, 256])
            ret = cv2.compareHist(target_hist, compare_hist, 0)
            print("{}: {}".format(i, ret))
            if ret > max_sim_point:
                ans_num = i
                max_sim_point = ret
        return ans_num
    
    def debug_view(self, diff_x, diff_y, ans_lst):
        ideal_img = copy.deepcopy(self.ideal_img)
        for i, ga_pos in enumerate(self.ga_calced_pos):
            h, w, _c = self.input_imgs[i].shape
            print(h,w)
            lt = (int(ga_pos[0] - w/2), int(ga_pos[1] - h/2))
            lb = (int(ga_pos[0] - w/2), int(ga_pos[1] + h/2))
            rt = (int(ga_pos[0] + w/2), int(ga_pos[1] - h/2))
            rb = (int(ga_pos[0] + w/2), int(ga_pos[1] + h/2))
            ideal_img = self.debug_draw_line(ideal_img, lt, lb, rt, rb, color=(0, 255, 0))
            # ideal_img = cv2.drawMarker(ideal_img, (int(ga_pos[0]), int(ga_pos[1])), color=(255, 0, 0))
        # cv2.imwrite("img/1106/result_debug_0.jpg", ideal_img)
        result_img = copy.deepcopy(self.result_img)
        for xs, ys in zip(self.xs_lst, self.ys_lst):
            lt, lb, rt, rb = self.reorder_point(xs, ys)
            result_img = self.debug_draw_line(result_img, lt, lb, rt, rb, color=(255, 0, 0))
        afin_matrix = np.float32([[1, 0, diff_x], [0, 1, diff_y]])
        ideal_img = cv2.warpAffine(ideal_img, afin_matrix, (640, 480))
        blended_img = cv2.addWeighted(src1=result_img, alpha=0.6, src2=ideal_img, beta=0.4, gamma=0)
        for i, ans in enumerate(ans_lst):
            pt1 = (int(sum(self.xs_lst[i])/4.0), int(sum(self.ys_lst[i])/4.0))
            pt2 = (int(ans[0]), int(ans[1]))
            print("pt1: {}, pt2: {}".format(pt1, pt2))
            cv2.arrowedLine(blended_img, pt1, pt2, (0, 0, 255), thickness=2, tipLength=0.3)
        cv2.imwrite("img/1106/result_debug.jpg", blended_img)

    def check_status_main(self, box_num):
        self.clip_found_decoration()
        ans_lst = [(-1, -1)] * len(self.decorated_imgs)
        cur_operate_deco = self.input_imgs[box_num]  # cv2.imread("img/1106/input" + str(box_num) + ".jpg")
        match_num = self.get_similar_img_num(cur_operate_deco, self.decorated_imgs)
        if match_num == -1:
            return ans_lst
        # 今ロボットが飾り付けた画像を基準に他の飾りのずれを見る
        ga_pos_x, ga_pos_y = int(self.ga_calced_pos[box_num][0]), int(self.ga_calced_pos[box_num][1])
        real_pos_x = int(sum(self.xs_lst[match_num]) / 4.0)
        real_pos_y = int(sum(self.ys_lst[match_num]) / 4.0)
        diff_x, diff_y = real_pos_x - ga_pos_x, real_pos_y - ga_pos_y

        for i in range(len(self.input_imgs)):
            match_num = self.get_similar_img_num(self.input_imgs[i], self.decorated_imgs)
            if match_num != -1:
                ideal_x = self.ga_calced_pos[i][0] + diff_x
                ideal_y = self.ga_calced_pos[i][1] + diff_y
                ans_lst[match_num] = (ideal_x, ideal_y)
        # print_result
        for i, ans in enumerate(ans_lst):
            print("decorated_{}: {}, {} -> {}, {}".format(i,
                    int(sum(self.xs_lst[i])/4.0), int(sum(self.ys_lst[i])/4.0), int(ans[0]), int(ans[1])))
        # 帰り値は、今壁上にある各boxに対して、GAで計算した本来置かれるべき場所
        self.debug_view(diff_x, diff_y, ans_lst)

check_deco_status = CheckDecoStatus()
check_deco_status.check_status_main(1)