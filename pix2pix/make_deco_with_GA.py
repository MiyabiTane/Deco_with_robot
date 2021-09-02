#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import glob
import numpy as np
import subprocess
import random
from copy import deepcopy
# import skimage.colo
from collections import deque

TH = 150
ALPHA = 0
INPUT_PATH = "data/balloon/*.jpg"


def think_with_trained_pix2pix(input_name, output_name):
    input_img = cv2.imread(input_name)  # input_img.shape = (480, 640, 3)
    img = np.full((640, 640, 3), 255)
    img[90: 570] = input_img
    save_name = input_name.rsplit(".")[0] + "_i.jpg"
    cv2.imwrite(save_name, img)
    # 学習済みpix2pixによる飾り付け画像生成
    subprocess.run(["pipenv", "run", "python", "trained_pix2pix.py", "--input", save_name, "--output", output_name])
    output_img = cv2.imread(output_name)
    output_img = cv2.resize(output_img , (640, 640))
    output_img = output_img[90: 570]
    cv2.imwrite(output_name, output_img)


def get_inputs():
    deco_imgs = []
    files = glob.glob("input_gan_images/input*.jpg")
    files = sorted(files, key=lambda x: int(x[-5]))
    for fi in files:
        img = cv2.imread(fi)
        deco_imgs.append(img)
    deco_masks = []
    files = glob.glob("input_gan_images/mask*.jpg")
    files = sorted(files, key=lambda x: int(x[-5]))
    for fi in files:
        img = cv2.imread(fi, 0)
        deco_masks.append(img)
    input_img = cv2.imread("input_gan_images/back.jpg")
    output_img = cv2.imread("input_gan_images/pix2pix.jpg")
    return deco_imgs, deco_masks, input_img, output_img


class ThinkDecoration:
    def __init__(self, deco_imgs, deco_masks, input_img, output_img, nums=21, generation=30, elite=2):
        # 複数の飾りが重ならないようにする 0: 空きスペース, 1: 飾りが既にある, 2: 飾りが既にあり、書き換え不可能
        self.visited = np.zeros((480, 640), dtype=np.int)
        self.input = input_img
        self.H, self.W, _ = self.input.shape
        self.output = output_img
        self.imgs = deco_imgs
        self.masks = deco_masks
        self.nums = nums
        self.elite = elite
        self.generation = generation
        # 貼り付け位置
        self.genes = []
        self.best_gene = (-1 * float('inf'), None)
        for _ in range(nums):
            gene = []
            for im in self.imgs:
                h, w, _ = im.shape
                random_x = random.randint(w / 2, self.W - w / 2)
                random_y = random.randint(h / 2, self.H - h / 2)
                # print(h, w, random_x, random_y)
                gene.append((random_x, random_y, h, w))
            gene = self.remove_overlap(gene)
            self.genes.append(gene)
            output_img = self.generate_img(gene)
        # print(self.genes)


    def shift_pos(self, pos_x, pos_y, h, w):
        if pos_x - w / 2 < 0:
            pos_x = int(w / 2)
        elif pos_x + w / 2 > self.W:
            pos_x = int(self.W - w / 2)
        if pos_y - h / 2 < 0:
            pos_y = int(h / 2)
        elif pos_y + h / 2 >= self.H:
            pos_y = int(self.H - h / 2)
        return pos_x, pos_y


    def remove_overlap(self, gene):
        new_gene = []
        self.visited = np.where(self.visited == 1, 0, self.visited)
        for pos_x, pos_y, h, w in gene:
            new_x, new_y = self.generate_new_pos(pos_x, pos_y, h, w)
            self.visited[int(new_y - h/2): int(new_y + h/2), int(new_x - w/2): int(new_x + w/2)] = 1
            new_gene.append((new_x, new_y, h, w, ))
        return new_gene


    def generate_new_pos(self, pos_x, pos_y, h, w):
        to_visit = deque([(pos_x, pos_y)])
        while to_visit:
            pos_x, pos_y = to_visit.popleft()
            check_array = self.visited[int(pos_y - h/2): int(pos_y + h/2), int(pos_x - w/2): int(pos_x + w/2)]
            over_y, over_x = np.where((check_array == 1) | (check_array == 2))
            over_y, over_x = set(over_y), set(over_x)
            if len(over_x) == 0 and len(over_y) == 0:
                return pos_x, pos_y
            # 右側にずらした時の座標
            ans_x = pos_x + max(over_x) + 1
            if ans_x + w / 2 <= self.W:
                to_visit.append((ans_x, pos_y))
            # 左側にずらした時の座標
            ans_x = pos_x - (w - min(over_x))
            if ans_x - w / 2 >= 0:
                to_visit.append((ans_x, pos_y))
            # 下側側にずらした時の座標
            ans_y = pos_y + max(over_y) + 1
            if ans_y + h / 2 <= self.H:
                to_visit.append((pos_x, ans_y))
            # 上側にずらした時の座標
            ans_y = pos_y - (h - min(over_y))
            if ans_y - h / 2 >= 0:
                to_visit.append((pos_x, ans_y))
        # どこへもずらせない時は画像の右下に配置
        return int(self.W - w / 2), int(self.H - h / 2)


    def generate_img(self, gene):
        # self.visited = np.where(self.visited == 1, 0, self.visited))
        output_img = deepcopy(self.input)
        for i, deco_img in enumerate(self.imgs):
            pos_x, pos_y, h, w = gene[i]
            mask_img = self.masks[i]
            back_img = output_img[int(pos_y - h/2): int(pos_y + h/2), int(pos_x - w/2): int(pos_x + w/2)]
            deco_img[mask_img < 150] = [0, 0, 0]
            back_img[mask_img >= 150] = [0, 0, 0]
            comp_img = cv2.add(deco_img, back_img)
            output_img[int(pos_y - h/2): int(pos_y + h/2), int(pos_x - w/2): int(pos_x + w/2)] = comp_img
        # cv2.imwrite("output0707.jpg", output_img)
        return output_img


    def evaluate(self):
        points = []
        for i in range(self.nums):
            decorated_img = self.generate_img(self.genes[i])
            # 各飾りに関して、置く前と置いた後のどちらが類似度が高いかを調べる
            diff = 0
            for pos_x, pos_y, h, w in self.genes[i]:
                ly, ry, lx, rx = int(pos_y - h/2), int(pos_y + h/2), int(pos_x - w/2), int(pos_x + w/2)
                before_diff = np.sum(np.linalg.norm(self.input[ly: ry, lx: rx, :] - self.output[ly: ry, lx: rx, :], axis=0))
                after_diff = np.sum(np.linalg.norm(decorated_img[ly: ry, lx: rx, :] - self.output[ly: ry, lx: rx, :], axis=0))
                diff += before_diff - after_diff  # 飾りを置くことでdiffが小さくなっていれば良い
            points.append(diff)
        points = np.array(points)
        print(max(points))
        if self.best_gene[0] < max(points):
            best_index = np.argsort(points)[-1]
            self.best_gene = (points[best_index], self.genes[best_index])
            # print(self.best_gene)
        points = points - min(points) if min(points) < 0 else points
        points = np.array([1/len(points)] * len(points)) if sum(points) == 0 else points / sum(points)
        return points


    def partial_crossover(self, parent_1, parent_2):
        num = len(parent_1)
        cross_point = random.randrange(2, num-1)
        child_1 = parent_1
        child_2 = parent_2
        for i in range(num - cross_point):
            target_index = cross_point + i
            x1, y1, h, w = parent_1[target_index]
            x2, y2, _h, _w = parent_2[target_index]
            min_x, max_x = min(x1, x2), max(x1, x2)
            min_y, max_y = min(y1, y2), max(y1, y2)
            min_nx = int(min_x - ALPHA * random.random() * (max_x - min_x))
            max_nx = int(max_x + ALPHA * random.random() * (max_x - min_x))
            min_ny = int(min_y - ALPHA * random.random() * (max_y - min_y))
            max_ny = int(max_y + ALPHA * random.random() * (max_y - min_y))
            p1_x = random.randint(min_nx, max_nx)
            p2_x = random.randint(min_nx, max_nx)
            p1_y = random.randint(min_ny, max_ny)
            p2_y = random.randint(min_ny, max_ny)
            p1_x, p1_y = self.shift_pos(p1_x, p1_y, h, w)
            p2_x, p2_y = self.shift_pos(p2_x, p2_y, h, w)
            child_1[target_index] = (p1_x, p1_y, h, w)
            child_2[target_index] = (p2_x, p2_y, h, w)
        child_1 = self.remove_overlap(child_1)
        child_2 = self.remove_overlap(child_2)
        return child_1, child_2
        # ブレンド交叉
        # https://qiita.com/simanezumi1989/items/4f821de2b77850fcf508


    def generate_next_generation(self):
        points = self.evaluate()
        copy = deepcopy(self.genes)
        for i in range((self.nums - self.elite)//2):
            index_1, index_2 = np.random.choice(len(points), 2, replace = True, p = points)
            #print(index_1, index_2)
            parent_1 = self.genes[index_1]
            parent_2 = self.genes[index_2]
            child_1, child_2 = self.partial_crossover(parent_1, parent_2)
            copy[2*i] = child_1
            copy[2*i + 1] = child_2
        # inherit high point parents
        elite_parent_index = np.argsort(points)
        for i in range(self.nums - self.elite, self.nums):
            copy[i] = self.genes[elite_parent_index[i]]
        self.genes = deepcopy(copy)


    def mutation(self, num_mutation = 3, mute_per = 0.7):
    # num_mutaiton can mutate at the percentage of mute_per 
        mutation_genes = np.random.choice(len(self.genes), num_mutation, replace = False)
        for index in mutation_genes:
            flag = np.random.choice(2, 1, p = [1 - mute_per, mute_per])
            if flag == 1:
                m_index = np.random.choice(len(self.imgs), 1, replace = False)[0]
                h, w, _ = self.imgs[m_index].shape
                random_x = random.randint(w / 2, self.W - w / 2)
                random_y = random.randint(h / 2, self.H - h / 2)
                self.genes[index][m_index] == (random_x, random_y, h, w)
                self.genes[index] = self.remove_overlap(self.genes[index])


    def GA_calc(self):
        for _ in range(self.generation):
            self.generate_next_generation()
            self.mutation()
            # print(self.genes)
        best_point, best_gene = self.best_gene
        print("BEST POINT: ", best_point)
        output_img = self.generate_img(best_gene)
        cv2.imwrite("ga_output.jpg", output_img)


# think_with_trained_pix2pix("input_gan_images/back.jpg", "input_gan_images/pix2pix.jpg")
deco_imgs, deco_masks, input_img, output_img = get_inputs()
think_deco = ThinkDecoration(deco_imgs, deco_masks, input_img, output_img)
think_deco.GA_calc()


# GA http://samuiui.com/2019/10/27/python%E3%81%A7%E9%81%BA%E4%BC%9D%E7%9A%84%E3%82%A2%E3%83%AB%E3%82%B4%E3%83%AA%E3%82%BA%E3%83%A0%EF%BC%88ga%EF%BC%89%E3%82%92%E5%AE%9F%E8%A3%85%E3%81%97%E3%81%A6%E5%B7%A1%E5%9B%9E%E3%82%BB%E3%83%BC/
