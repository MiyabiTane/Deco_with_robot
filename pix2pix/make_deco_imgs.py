"""s
... thinking decoration ...
Back Img info->
('lt: ', 1932.3653923846878, 998.3951007831643, 1784.9823432069206)
('rb: ', 1927.2010199713968, -1260.9923844731593, 40.04882008628147)
('head_angle: ', 32.31637692749683)
('look_at_point: ', x: 1068.30170515
y: -172.769141042
z: 653.992004864)
('look_at_uv: ', x: 312.801434704
y: 292.505059252
z: 0.0)
('dimg_rect_pos: ', array([x: 2147.39202783
y: 672.95587784
z: 1076.35317635,
       x: 886.685198837
y: -1078.03702755
z: 4.57065396599], dtype=object))
= decos_pos
[x: 1006.33938725
y: -667.517274584
z: 653.77801336
 x: 1136.28470115
y: 321.170410664
z: 653.814447653
 x: 1071.88769772
y: -173.292084519
z: 653.996440072]
= decos_dims
[x: 211.286127567
y: 207.858324051
z: 89.3031358719
 x: 211.575657129
y: 207.611441612
z: 89.3883705139
 x: 151.288926601
y: 152.40624547
z: 89.7635817528]
= decos_rec_uv
[ header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
xs: [473.3926696777344, 555.76123046875, 510.2060852050781, 603.1673583984375]
ys: [250.5164794921875, 243.0118408203125, 308.7667541503906, 299.02618408203125]
type: 0
label: ''
fit_line: False
fit_line_ransac: False
 header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
xs: [49.51151657104492, 140.52293395996094, 26.32427215576172, 130.5523681640625]
ys: [255.45819091796875, 247.78318786621094, 315.3082580566406, 305.3094787597656]
type: 0
label: ''
fit_line: False
fit_line_ransac: False
 header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
xs: [275.5888671875, 340.2502746582031, 281.9150390625, 352.7976379394531]
ys: [259.103271484375, 253.34063720703125, 301.0009460449219, 294.0481872558594]
type: 0
label: ''
fit_line: False
fit_line_ransac: False]
"""

import cv2
import copy
import numpy as np

class MakeDecoImgs(object):
    def __init__(self):
        self.bimg_lt_3d_pos = [1932.3653923846878, 998.3951007831643, 1784.9823432069206]
        self.bimg_rb_3d_pos = [1927.2010199713968, -1260.9923844731593, 40.04882008628147]
        self.head_angle = 32.31637692749683
        self.look_at_point = [1068.30170515, -172.769141042, 653.992004864]
        self.look_at_uv = [312.801434704, 292.505059252, 0.0]
        self.dimg_lt_3d = [2147.39202783, 672.95587784, 1076.35317635]
        self.dimg_rb_3d = [886.685198837, -1078.03702755, 4.57065396599]
        self.deco_3d_pos = [[1006.33938725, -667.517274584, 653.77801336],
                            [1136.28470115, 321.170410664, 653.814447653],
                            [1071.88769772, -173.292084519, 653.996440072]]
        self.decos_3d_dims = [[211.286127567, 207.858324051, 89.3031358719],
                              [211.575657129, 207.611441612, 89.3883705139],
                              [151.288926601, 152.40624547, 89.7635817528]]
        self.deco0_xs = [473.3926696777344, 555.76123046875, 510.2060852050781, 603.1673583984375]
        self.deco0_ys = [250.5164794921875, 243.0118408203125, 308.7667541503906, 299.02618408203125]
        self.deco1_xs = [49.51151657104492, 140.52293395996094, 26.32427215576172, 130.5523681640625]
        self.deco1_ys = [255.45819091796875, 247.78318786621094, 315.3082580566406, 305.3094787597656]
        self.deco2_xs = [275.5888671875, 340.2502746582031, 281.9150390625, 352.7976379394531]
        self.deco2_ys = [259.103271484375, 253.34063720703125, 301.0009460449219, 294.0481872558594]

        self.back_img = cv2.imread("img_from_gazebo/input.png")
        self.decos_img = cv2.imread("img_from_gazebo/decos_img.jpg")

        self.camera_pos = []
        self.view_vec = []
        self.R_matrix = []
        self.M_matrix = []

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

    def draw_line(self, img, xs, ys):
        lt, lb, rt, rb = self.reorder_point(xs, ys)
        draw_point = [lt, rt, rb, lb]
        for i in range(len(draw_point)):
            n_i = 0 if i + 1 == len(xs) else i + 1
            cv2.line(img, draw_point[i], draw_point[n_i], (255, 255, 255), thickness=1, lineType=cv2.LINE_4)
        return img

    def debug_draw_deco_rect(self, input_img):
        vis_decos_img = copy.deepcopy(input_img)
        vis_decos_img = self.draw_line(vis_decos_img, self.deco0_xs, self.deco0_ys)
        vis_decos_img = self.draw_line(vis_decos_img, self.deco1_xs, self.deco1_ys)
        vis_decos_img = self.draw_line(vis_decos_img, self.deco2_xs, self.deco2_ys)
        cv2.imwrite("img_from_gazebo/decos_vis_img.jpg", vis_decos_img)
    
    def make_view_point(self):
        xy_length = np.linalg.norm(np.array([self.look_at_point[0], self.look_at_point[1], 0]))
        camera_z = self.look_at_point[2] + xy_length / np.tan(self.head_angle * np.pi / 180.0)
        self.camera_pos = np.array([0, 0, camera_z])
        print("camera pos: ", self.camera_pos)
        self.view_vec = np.array(self.look_at_point) - self.camera_pos
        print("view_vec: ", self.view_vec)

    def make_rotation_matrix(self):
        # https://yttm-work.jp/gmpg/gmpg_0003.html
        self.make_view_point()
        camera_z = self.view_vec / np.linalg.norm(self.view_vec)
        camera_x = np.cross(np.array([0, 1, 0]), camera_z)
        camera_x = camera_x / np.linalg.norm(camera_x)
        camera_y = np.cross(camera_z, camera_x)
        camera_y = camera_y / np.linalg.norm(camera_y)

        rotation_matrix = np.zeros((4, 4))
        rotation_matrix[0][:3] = camera_x
        rotation_matrix[1][:3] = camera_y
        rotation_matrix[2][:3] = camera_z
        rotation_matrix[3][3] = 1.0
        self.R_matrix = rotation_matrix.T 
        print(self.R_matrix)
    
    def world_to_camera_pos(self, xyz_array):
        input_pos = np.ones((1, 4))
        input_pos[0][:3] = xyz_array - self.camera_pos
        output_pos = np.matmul(np.linalg.inv(self.R_matrix), input_pos.T)
        return np.array(output_pos[:3])
    
    def camera_3d_to_2d(self, xyz_array):
        # robot x, y -> img -y, -x
        x = int(-1 * xyz_array[1])
        y = int(xyz_array[0])
        return np.array([x, y])

    def convert_img_all(self):
        """
        convert the image to one viewed from the camera directly above
        world -> camera
            self.world_to_camera_pos(input_pos)
        camera -> world
            center_pos = np.matmul(self.R_matrix, input_pos.T)
        """
        camera_lt = self.world_to_camera_pos(self.dimg_lt_3d)
        camera_rb = self.world_to_camera_pos(self.dimg_rb_3d)
        center_pos = self.world_to_camera_pos(self.look_at_point)
        img_lt = self.camera_3d_to_2d(camera_lt) + np.array(self.look_at_uv[:2])
        img_rt = np.array([img_lt[0] * -1, img_lt[1]]) + np.array(self.look_at_uv[:2])
        img_rb = self.camera_3d_to_2d(camera_rb) + np.array(self.look_at_uv[:2])
        img_lb = np.array([img_rb[0] * -1, img_rb[1]]) + np.array(self.look_at_uv[:2])
        print(center_pos)
        print(camera_lt)
        print(camera_rb)
        print("")
        print(img_lt)
        print(img_rt)
        print(img_rb)
        print(img_lb)

        pts1 = np.float32([img_lt, img_rt, img_lb, img_rb])
        pts2 = np.float32([[0, 0], [640, 0], [0, 480], [640, 480]])
        self.M_matrix = cv2.getPerspectiveTransform(pts1,pts2)
        decos_img_trans = cv2.warpPerspective(self.decos_img, self.M_matrix, (640,480))
        cv2.imwrite("img_from_gazebo/decos_convert_img.jpg", decos_img_trans)

    def convert_img_deco(self):
        decos_3d_pos = self.deco_3d_pos[0]
        decos_3d_dims = self.decos_3d_dims[0]
        xs = self.deco0_xs
        ys = self.deco0_ys
        lt, lb, rt, rb = self.reorder_point(xs, ys)
        H = int(decos_3d_dims[0])
        W = int(decos_3d_dims[1])
        pts1 = np.float32([lt, rt, lb, rb])
        pts2 = np.float32([[0, 0], [W, 0], [0, H], [W, H]])
        self.M_matrix = cv2.getPerspectiveTransform(pts1, pts2)
        decos_img_trans = cv2.warpPerspective(self.decos_img, self.M_matrix, (W, H))
        cv2.imwrite("img_from_gazebo/decos_convert_img.jpg", decos_img_trans)

    def convert_img_size(self):  #, deco_3d_dims):
        img = cv2.imread("img_from_gazebo/decos_convert_img.jpg")
        H, W, C = img.shape
        decos_3d_dims = self.decos_3d_dims[0]
        bimg_width = self.bimg_lt_3d_pos[1] - self.bimg_rb_3d_pos[1]
        bimg_height = self.bimg_lt_3d_pos[2] - self.bimg_rb_3d_pos[2]
        ratio_w = decos_3d_dims[1] / bimg_width
        ratio_h = decos_3d_dims[0] / bimg_height
        deco_w = int(640 * ratio_w)
        deco_h = int(480 * ratio_h)
        dst = cv2.resize(img, dsize=(deco_w, deco_h))
        cv2.imwrite("img_from_gazebo/decos_convert_img.jpg", dst)

        # make mask img
        img_white = np.ones((deco_w, deco_h), np.uint8) * 255
        cv2.imwrite("img_from_gazebo/mask.jpg", img_white)

    def convert_rect_pos(self):
        for i in range(len(self.deco0_xs)):
            output_pos = np.matmul(self.M_matrix, np.array([self.deco0_xs[i], self.deco0_ys[i], 1]).T)
            self.deco0_xs[i] = output_pos[0]
            self.deco0_ys[i] = output_pos[1]
        for i in range(len(self.deco1_xs)):
            output_pos = np.matmul(self.M_matrix, np.array([self.deco1_xs[i], self.deco1_ys[i], 1]).T)
            self.deco1_xs[i] = output_pos[0]
            self.deco1_ys[i] = output_pos[1]       
        for i in range(len(self.deco2_xs)):
            output_pos = np.matmul(self.M_matrix, np.array([self.deco2_xs[i], self.deco2_ys[i], 1]).T)
            self.deco2_xs[i] = output_pos[0]
            self.deco2_ys[i] = output_pos[1]
        input_img = cv2.imread("img_from_gazebo/decos_convert_img.jpg")
        self.debug_draw_deco_rect(input_img)
    
    def clip_img(self, img, xs_lst, ys_lst):
        left = int(min(xs_lst))
        up = int(min(ys_lst))
        right = int(max(xs_lst))
        bottom = int(max(ys_lst))
        return img[up: bottom, left: right, :]

make_deco_imgs = MakeDecoImgs()
make_deco_imgs.debug_draw_deco_rect(make_deco_imgs.decos_img)
make_deco_imgs.make_rotation_matrix()
make_deco_imgs.convert_img_deco()
make_deco_imgs.convert_img_size()
