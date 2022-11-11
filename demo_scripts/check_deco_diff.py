import glob
import numpy as np
import cv2
from copy import deepcopy

IDEAL_IMAGE_PATH = "img/ga_output_0.jpg"
REAL_IMAGE_PATH = "img/deco_result.jpg"
INIT_IMAGE_PATH = "img/back_img.jpg"
DECO_IMAGE_PATH = "img/input0.jpg"
NOT_FOUND_TH = 0.0

def debug_visualize_line(img, cx, cy, deco_img, save_name):
     output_debug_img = deepcopy(img)
     h, w, _ = deco_img.shape
     lx, ly = int(cx - w / 2.0), int(cy - h / 2.0)
     rx, ry = int(cx + w / 2.0), int(cy + h / 2.0)
     cv2.rectangle(output_debug_img, (lx, ly), (rx, ry), (255, 0, 0), thickness=2, lineType=cv2.LINE_4)
     cv2.imwrite(save_name, output_debug_img)

def get_match_pos(back_img, deco_img, thresh):
     res = cv2.matchTemplate(back_img, deco_img, cv2.TM_CCOEFF_NORMED)
     _min_val, max_val, _min_loc, max_loc = cv2.minMaxLoc(res)
     if max_val > thresh:
          H, W, _ = deco_img.shape
          lx, ly = max_loc
          cx = int(lx + W / 2.0)
          cy = int(ly + H / 2.0)
          # self.debug_visualize_line(back_img, cx, cy, deco_img)
          return cx, cy
     return -1, -1

def check_diff():
     thresh = 1.1
     ga_back_img = cv2.imread(IDEAL_IMAGE_PATH)
     real_back_img = cv2.imread(REAL_IMAGE_PATH)
     deco_img = cv2.imread(DECO_IMAGE_PATH)
     ga_x, ga_y, res_x, res_y = -1, -1, -1, -1
     while -1 in [ga_x, ga_y, res_x, res_y] and thresh >= NOT_FOUND_TH:
          thresh -= 0.05
          print("thresh: ", thresh)
          ga_x, ga_y = get_match_pos(ga_back_img, deco_img, thresh)
          if not -1 in [ga_x, ga_y]:
               debug_visualize_line(ga_back_img, ga_x, ga_y, deco_img, "img/debug_ideal.jpg")
          res_x, res_y = get_match_pos(real_back_img, deco_img, thresh)
          if not -1 in [ga_x, ga_y]:
               debug_visualize_line(real_back_img, res_x, res_y, deco_img, "img/debug_real.jpg")
          # print("ga_x, ga_y, res_x, res_y", ga_x, ga_y, res_x, res_y)
     if thresh < NOT_FOUND_TH:
          print("Fail Decorate")
     else:
          afin_matrix = np.float32([[1, 0, ga_x - res_x], [0, 1, ga_y - res_y]])
          res_img = cv2.warpAffine(real_back_img, afin_matrix, (640, 480))
          deco_files = glob.glob("img/input*.jpg")
          for i in range(len(deco_files)):
               fi = "img/input" + str(i) + ".jpg"
               deco_img = cv2.imread(fi)
               ga_x, ga_y = get_match_pos(ga_back_img, deco_img, thresh)
               if not -1 in [ga_x, ga_y]:
                    debug_visualize_line(ga_back_img, ga_x, ga_y, deco_img, "img/debug_ideal_" + str(i) + ".jpg")
               res_x, res_y = get_match_pos(res_img, deco_img, thresh)
               if not -1 in [res_x, res_y]:
                    debug_visualize_line(res_img, res_x, res_y, deco_img, "img/debug_real_" + str(i) + ".jpg")
               print("({}, {}) -> ({}, {})".format(ga_x, ga_y, res_x, res_y))

TH = 50
KER_1 = 5
KER_2 = 5

def get_diff_img(before_img, after_img, k_size=KER_1, k_size2=KER_2):
     im_diff = before_img.astype(int) - after_img.astype(int)
     im_diff_abs = np.abs(im_diff)
     im_diff_img = im_diff_abs.astype(np.uint8)
     im_diff_img[np.where(im_diff_abs[:,:,0] < TH) and np.where(im_diff_abs[:,:,1] < TH) and np.where(im_diff_abs[:,:,2] < TH)] = [0, 0, 0]
     img_gray = cv2.cvtColor(im_diff_img, cv2.COLOR_BGR2GRAY)
     _, img_binary = cv2.threshold(img_gray, 1, 255, cv2.THRESH_BINARY)
     # remove noise
     kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(k_size,k_size))
     kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT,(k_size2,k_size2))
     img_opening = cv2.morphologyEx(img_binary, cv2.MORPH_OPEN, kernel)
     img_closing = cv2.morphologyEx(img_opening, cv2.MORPH_CLOSE, kernel2)
     cv2.imwrite("img/diff_img.jpg", img_closing)
     return img_closing

def get_deco_rect(diff_img):
     lst = []
     thresh_size = 3000
     contours, _hierarchy = cv2.findContours(diff_img ,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
     for cnt in contours:
          if cv2.contourArea(cnt) > thresh_size:
               print(cv2.contourArea(cnt))
               # rect = cv2.minAreaRect(cnt)
               # box = cv2.boxPoints(rect)
               # box = np.int0(box)
               x, y, w, h = cv2.boundingRect(cnt)
               lst.append((x, y, w, h))
     for info in lst:
          lx, ly, w, h = info
          cv2.rectangle(diff_img, (lx, ly), (lx + w, ly + h), (255, 0, 0), thickness=2, lineType=cv2.LINE_4)
     cv2.imwrite("img/diff_img.jpg", diff_img)


before_img = cv2.imread(INIT_IMAGE_PATH)
after_img = cv2.imread(REAL_IMAGE_PATH)
diff_img = get_diff_img(before_img, after_img)
get_deco_rect(diff_img)
# check_diff()

