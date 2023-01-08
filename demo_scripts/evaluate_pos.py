import cv2
import numpy as np
import matplotlib.pyplot as plt

class GetClickedPos:
    def __init__(self):
        self.clicked_pos = []

    def get_position(self, img_name):
        def onMouse(event, x, y, flags, params):
            if event == cv2.EVENT_LBUTTONDOWN:
                print("clicked pos (x,y) =", x, y)
                self.clicked_pos.append([x, y])
        img = cv2.imread(img_name, cv2.IMREAD_COLOR)
        cv2.namedWindow('window', cv2.WINDOW_NORMAL)
        cv2.imshow('window', img)
        cv2.setMouseCallback('window', onMouse)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print("clicked_pos: ", self.clicked_pos)
 

def convert_img(ga_img_name, res_img_name):
    print("2点をクリック")
    print("クリックが終わったらキーボードを押して下さい")
    get_cpos = GetClickedPos()
    get_cpos.get_position(ga_img_name)
    t_1, b_1 = get_cpos.clicked_pos

    print("2点をクリック")
    print("クリックが終わったらキーボードを押して下さい")
    get_cpos = GetClickedPos()
    get_cpos.get_position(res_img_name)
    t_2, b_2 = get_cpos.clicked_pos

    resize_scale = (b_1[1] - t_1[1]) / float(b_2[1] - t_2[1])
    print("resize_scale: ", resize_scale)
    res_img = cv2.imread(res_img_name)
    res_img = cv2.resize(res_img, dsize=None, fx=resize_scale, fy=resize_scale)
    cv2.imwrite("scaled.jpg", res_img)

    print("2点をクリック")
    print("クリックが終わったらキーボードを押して下さい")
    get_cpos = GetClickedPos()
    get_cpos.get_position("scaled.jpg")
    t_2, b_2 = get_cpos.clicked_pos

    dx = ((t_1[0] - t_2[0]) + (b_1[0] - b_2[0])) / 2.0
    dy = ((t_1[1] - t_2[1]) + (b_1[1] - b_2[1])) / 2.0
    print("dx: ", dx, "dy: ", dy)
    affine_mat = np.float32([[1, 0, dx], [0, 1, dy]])
    res_img = cv2.warpAffine(res_img, affine_mat, (640, 480))

    cv2.imwrite("result.jpg", res_img)


def check_diff(ga_pos, ga_img_name, res_img_name, save_name):
    print("風船をクリックして、キーボードを押して下さい")
    get_cpos = GetClickedPos()
    get_cpos.get_position(res_img_name)
    res_deco_pos = get_cpos.clicked_pos[0]
    print("Result Deco Pos: ", res_deco_pos)

    ga_img = cv2.imread(ga_img_name)
    res_img = cv2.imread(res_img_name)
    blended_img = cv2.addWeighted(src1=ga_img, alpha=0.6, src2=res_img, beta=0.4, gamma=0)
    pt1 = (int(ga_pos[0]), int(ga_pos[1]))
    pt2 = (int(res_deco_pos[0]), int(res_deco_pos[1]))
    cv2.arrowedLine(blended_img, pt1, pt2, (0, 0, 255), thickness=2, tipLength=0.3)
    print("Diff_pos: ", pt2[0] - pt1[0], pt2[1] - pt1[1])
    cv2.imwrite(save_name, blended_img)

convert_img("ga_output_0.jpg", "back_img_0.jpg")
check_diff([286.0, 310.0], "ga_output_0.jpg", "result.jpg", "result_0.jpg")
convert_img("ga_output_1.jpg", "back_img_1.jpg")
check_diff([346.0, 359.0], "ga_output_1.jpg", "result.jpg", "result_1.jpg")
convert_img("ga_output_2.jpg", "back_img_2.jpg")
check_diff([359.0, 268.0], "ga_output_2.jpg", "result.jpg", "result_2.jpg")
