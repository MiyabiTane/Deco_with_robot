"""
('lt: ', 1932.3476365247489, 998.372717774108, 1784.875143483393)
('rb: ', 1927.2403589830274, -1261.042495069667, 39.911393436548224)
('head_angle: ', 29.662153031493034)
('look_at_point: ', x: 1180.10895533
y: -181.063904022
z: 654.224529006)
= decos_pos
[x: 1117.07664006
y: -678.027945277
z: 654.235697998
 x: 1253.48854073
y: 310.8842684
z: 654.134191761
 x: 1185.31627926
y: -182.925776119
z: 654.475951655]
= decos_dims
[x: 211.495101452
y: 210.085630417
z: 89.1058444977
 x: 211.82730794
y: 208.705306053
z: 89.1456007957
 x: 151.7547369
y: 152.527987957
z: 89.241206646]
= decos_req_uv
[ header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
xs: [463.5361633300781, 541.403564453125, 490.3731384277344, 575.0953979492188]
ys: [251.89376831054688, 245.59803771972656, 332.85540771484375, 324.47265625]
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
xs: [70.7800521850586, 155.0377655029297, 59.7741813659668, 152.2384796142578]
ys: [253.37588500976562, 247.0721435546875, 334.9308166503906, 326.5277404785156]
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
xs: [279.44622802734375, 339.48834228515625, 286.2708435058594, 349.6729736328125]
ys: [258.05303955078125, 253.30751037597656, 324.44281005859375, 318.5474853515625]
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
        self.bimg_lt_3d_pos = [1932.3476365247489, 998.372717774108, 1784.875143483393]
        self.bimg_rb_3d_pos = [1927.2403589830274, -1261.042495069667, 39.911393436548224]
        self.head_angle = 29.662153031493034
        self.look_at_point = [1180.10895533, -181.063904022, 654.224529006]
        self.deco_3d_pos = [[1117.07664006, -678.027945277, 654.235697998],
                            [1253.48854073, 310.8842684, 654.134191761],
                            [1185.31627926, -182.925776119, 654.475951655]]
        self.decos_3d_dims = [[211.495101452, 210.085630417, 89.1058444977],
                              [211.82730794, 208.705306053, 89.1456007957],
                              [151.7547369, 152.527987957, 89.241206646]]

        self.deco0_xs = [463.5361633300781, 541.403564453125, 490.3731384277344, 575.0953979492188]
        self.deco0_ys = [251.89376831054688, 245.59803771972656, 332.85540771484375, 324.47265625]
        self.deco1_xs = [70.7800521850586, 155.0377655029297, 59.7741813659668, 152.2384796142578]
        self.deco1_ys = [253.37588500976562, 247.0721435546875, 334.9308166503906, 326.5277404785156]
        self.deco2_xs = [279.44622802734375, 339.48834228515625, 286.2708435058594, 349.6729736328125]
        self.deco2_ys = [258.05303955078125, 253.30751037597656, 324.44281005859375, 318.5474853515625]

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
        return output_pos[:3]
    
    def camera_3d_to_2d(self, xyz_array):
        x = int(-1 * xyz_array[1])
        y = int(xyz_array[0])
        return [x, y]
    
    def convert_img(self):
        # robot x, y -> img -y, -x
        """
        convert the image to one viewed from the camera directly above
        world -> camera
            self.world_to_camera_pos(input_pos)
        camera -> world
            center_pos = np.matmul(np.linalg.inv(self.R_matrix), input_pos.T)
        """
        world_lt = self.look_at_point + np.array([240, 360, 0])
        world_rt = self.look_at_point + np.array([240, -360, 0])
        world_lb = self.look_at_point + np.array([-240, 360, 0])
        world_rb = self.look_at_point + np.array([-240, -360, 0])
        camera_lt = self.world_to_camera_pos(world_lt)
        camera_rt = self.world_to_camera_pos(world_rt)
        camera_lb = self.world_to_camera_pos(world_lb)
        camera_rb = self.world_to_camera_pos(world_rb)
        img_rt = self.camera_3d_to_2d(camera_rt - camera_lt)
        img_lb = self.camera_3d_to_2d(camera_lb - camera_lt)
        img_rb = self.camera_3d_to_2d(camera_rb - camera_lt)
        print(img_rt)
        print(img_lb)
        print(img_rb)

        pts1 = np.float32([[0, 0], img_rt, img_lb, img_rb])
        pts2 = np.float32([[0, 0], [640, 0], [0, 480], [640, 480]])
        self.M_matrix = cv2.getPerspectiveTransform(pts1,pts2)
        decos_img_trans = cv2.warpPerspective(self.decos_img, self.M_matrix, (640,480))
        cv2.imwrite("img_from_gazebo/decos_convert_img.jpg", decos_img_trans)

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
    
    def mask_img(self):
        print("ToDo")

make_deco_imgs = MakeDecoImgs()
make_deco_imgs.debug_draw_deco_rect(make_deco_imgs.decos_img)
make_deco_imgs.make_rotation_matrix()
make_deco_imgs.convert_img()
make_deco_imgs.convert_rect_pos()
converted_img = cv2.imread("img_from_gazebo/decos_convert_img.jpg")
img = make_deco_imgs.clip_img(converted_img, make_deco_imgs.deco0_xs, make_deco_imgs.deco0_ys)
cv2.imwrite("img_from_gazebo/output.jpg", img)
