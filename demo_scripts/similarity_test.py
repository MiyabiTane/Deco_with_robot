import cv2
import numpy as np
HIST_TH = 0.1
DISTANCE_TH = 50

def adjust_brightness(img, save_name):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)
    # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(3, 3))
    # result_v = clahe.apply(v)
    result_v = cv2.equalizeHist(v)
    hsv = cv2.merge((h, s, result_v))
    res_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    cv2.imwrite(save_name, res_img)
    return res_img

def get_average_color(img):
    h, w, c = img.shape
    color_arr = img.reshape(h * w, c)
    color_mean = np.mean(color_arr, axis=0)
    color_mean = np.array(color_mean) * 382 / np.sum(color_mean)
    color_mean = color_mean.astype(int)
    return color_mean

def make_debug_img(ave_rgb, save_name):
    res_img = np.ones((100, 100, 3)) * ave_rgb
    cv2.imwrite(save_name, res_img)

def get_sim_color_img_num(target_img, imgs, save_num=0):
    print("")
    ans_num = -1
    min_distance = DISTANCE_TH
    target_color = get_average_color(target_img)
    make_debug_img(target_color, "img/1120/output/" + str(save_num) + ".jpg")
    save_num += 1
    for i, img in enumerate(imgs):
        compare_color = get_average_color(img)
        make_debug_img(compare_color, "img/1120/output/" + str(save_num) + ".jpg")
        save_num += 1
        distance = np.linalg.norm(np.array(target_color) - np.array(compare_color))
        print("{}: {}  {} {}".format(i, distance, target_color, compare_color))
        if distance < min_distance:
            ans_num = i
            min_distance = distance
    print("matched_num: {}".format(ans_num))
    return ans_num

def get_similar_img_num(target_img, imgs, adjust_b=False, save_num=0):
    # use histgram of color
    print("")
    if adjust_b:
        target_img = adjust_brightness(target_img, "img/1120/output/" + str(save_num) + ".jpg")
        save_num += 1
    ans_num = -1
    max_sim_point = HIST_TH
    h, w, _ = target_img.shape
    target_hist = cv2.calcHist([target_img], [0], None, [256], [0, 256])
    for i, img in enumerate(imgs):
        if adjust_b:
            img = adjust_brightness(img, "img/1120/output/" + str(save_num) + ".jpg")
            save_num += 1
        img = cv2.resize(img, (w, h))
        compare_hist = cv2.calcHist([img], [0], None, [256], [0, 256])
        ret = cv2.compareHist(target_hist, compare_hist, 0)
        print("{}: {}".format(i, ret))
        if ret > max_sim_point:
            ans_num = i
            max_sim_point = ret
    print("matched_num: {}".format(ans_num))
    return ans_num


input0 = cv2.imread("img/1120/input0.jpg")
input1 = cv2.imread("img/1120/input1.jpg")
decorated0 = cv2.imread("img/1120/decorated0.jpg")
decorated1 = cv2.imread("img/1120/decorated1.jpg")
decorated2 = cv2.imread("img/1120/decorated2.jpg")

match_num = get_similar_img_num(input0, [decorated0, decorated1, decorated2])
if match_num == -1:
    get_sim_color_img_num(input0, [decorated0, decorated1, decorated2])

print("------------------")
match_num = -1
match_num = get_similar_img_num(input1, [decorated0, decorated1, decorated2])
if match_num == -1:
    get_sim_color_img_num(input1, [decorated0, decorated1, decorated2], save_num=10)

# get_similar_img_num(input0, [decorated0, decorated1, decorated2], adjust_b=True, save_num=0)
# get_similar_img_num(input1, [decorated0, decorated1, decorated2], adjust_b=True, save_num=10)
