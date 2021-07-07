#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import glob
import numpy as np
import subprocess
import random
from copy import deepcopy
import skimage.color
TH = 150
INPUT_PATH = "data/balloon/*.jpg"


def think_with_trained_pix2pix(input_name, output_name):
    input_img = cv2.imread(input_name)  # input_img.shape = (480, 640, 3)
    img = np.full((640, 640, 3), 255)
    img[90: 570] = input_img
    cv2.imwrite(input_name, img)
    # 学習済みpix2pixによる飾り付け画像生成
    subprocess.run(["pipenv", "run", "python", "trained_pix2pix.py", "--input", "input.jpg"])
    output_img = cv2.imread(output_name)
    output_img = cv2.resize(output_img , (640, 640))
    output_img = output_img[90: 570]
    cv2.imwrite(output_name, output_img)


def get_inputs():
    deco_imgs = []
    files = glob.glob("input_gan_images/input*.jpg")
    files = sorted(files, key=lambda x: int(x[-5]))
    print()
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
    def __init__(self, deco_imgs, deco_masks, input_img, output_img, nums=11, generation=10, elite=5):
        # self.visited = np.zeros((480, 640), dtype=np.int)
        # ToDo: 複数の飾りが重ならないようにする
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
        for _ in range(nums):
            gene = []
            for im in self.imgs:
                h, w, _ = im.shape
                random_x = random.randint(w / 2, self.W - w / 2)
                random_y = random.randint(h / 2, self.H - h / 2)
                gene.append((random_x, random_y))
            self.genes.append(gene)
        # print(self.genes)

    def generate_img(self, gene):
        output_img = deepcopy(self.input)
        for i, deco_img in enumerate(self.imgs):
            pos_x, pos_y = gene[i]
            mask_img = self.masks[i]
            h, w, _ = deco_img.shape
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
            target_color = skimage.color.rgb2lab(self.output)
            compare_color = skimage.color.rgb2lab(decorated_img)
            res = np.sum(skimage.color.deltaE_cie76(target_color, compare_color))
            points.append(1/res)  # 値が小さいほうが類似度が高い
        print(max(points))
        points = points / sum(points)
        return points


    def partial_crossover(self, parent_1, parent_2):
        num = len(parent_1)
        cross_point = random.randrange(2, num-1)
        child_1 = parent_1
        child_2 = parent_2
        for i in range(num - cross_point):
            target_index = cross_point + i
            value_1 = parent_1[target_index]
            value_2 = parent_2[target_index]
            child_1[target_index] = (int((value_1[0] * 2 + value_2[0]) / 3), int((value_1[1] * 2 + value_2[1]) / 3))
            child_2[target_index] = (int((value_1[0] + value_2[0] * 2) / 3), int((value_1[1] + value_2[1] * 2) / 3))
        return child_1, child_2


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
                self.genes[index][m_index] == (random_x, random_y)


    def GA_calc(self):
        for _ in range(self.generation):
            self.generate_next_generation()
            self.mutation()
        output_img = self.generate_img(self.genes[0])
        cv2.imwrite("ga_output.jpg", output_img)


deco_imgs, deco_masks, input_img, output_img = get_inputs()
think_deco = ThinkDecoration(deco_imgs, deco_masks, input_img, output_img)
# think_deco.evaluate()
think_deco.GA_calc()



# 画像の類似度を評価指標
# https://qiita.com/best_not_best/items/c9497ffb5240622ede01
# GAで遺伝子に各画像の貼り付け位置を与える
# http://samuiui.com/2019/10/27/python%E3%81%A7%E9%81%BA%E4%BC%9D%E7%9A%84%E3%82%A2%E3%83%AB%E3%82%B4%E3%83%AA%E3%82%BA%E3%83%A0%EF%BC%88ga%EF%BC%89%E3%82%92%E5%AE%9F%E8%A3%85%E3%81%97%E3%81%A6%E5%B7%A1%E5%9B%9E%E3%82%BB%E3%83%BC/
# https://pystyle.info/opencv-background-substraction/
# https://pctraveljournal.com/?p=704 
# bgObj =cv2.createBackgroundSubtractorMOG2()
# https://qiita.com/AtomJamesScott/items/ccef87b1092d7407de0d
# https://axa.biopapyrus.jp/ia/opencv/detect-contours.html
# https://www.higashisalary.com/entry/python-cv2-add
# https://zenn.dev/eetann/books/2020-09-25-make-mosaic-art-python/viewer/calculate-similar

