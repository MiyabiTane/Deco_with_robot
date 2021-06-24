import cv2
import glob
import subprocess
import os
import argparse
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--masked", action="store_true")
args = parser.parse_args()

input_path = glob.glob("input_pdf/*.pdf")
os.makedirs('images', exist_ok=True)
os.makedirs('images/A', exist_ok=True)
os.makedirs('images/B', exist_ok=True)
os.makedirs('images/output', exist_ok=True)
os.makedirs('images/keep', exist_ok=True)

for pdf in input_path:
    subprocess.run(["pdftoppm", "-jpeg", pdf, "images/keep/image"])
    type_ = pdf[-5]
    input_images = glob.glob("images/keep/*.jpg")
    for img_name in input_images:
        num = int(img_name[-6:-4])
        # print(img_name, num)
        img_ = cv2.imread(img_name)
        H, _W, _C = img_.shape
        img_ = img_[:, :H, :]
        small_img = cv2.resize(img_, (256, 256))
        # print(small_img.shape)
        save_name = str(num) if num > 9 else "0" + str(num)
        cv2.imwrite("images/" + type_ + "/" + save_name + ".jpg", small_img)

inputs_A = glob.glob("images/A/*.jpg")
inputs_A = sorted(inputs_A, key=lambda x: int(x[-6:-4]))
inputs_B = glob.glob("images/B/*.jpg")
inputs_B = sorted(inputs_B, key=lambda x: int(x[-6:-4]))
# print(inputs_A)
# print(inputs_B)

count = 0
for A_img, B_img in zip(inputs_A, inputs_B):
    print(A_img, B_img)
    imA = cv2.imread(A_img)
    imB = cv2.imread(B_img)
    if args.masked:
        imA_gray = cv2.cvtColor(imA, cv2.COLOR_BGR2GRAY)
        _, mask_img = cv2.threshold(imA_gray, 150, 255, cv2.THRESH_BINARY)
        # cv2.imwrite("images/output/" + str(count) + "_gray.jpg", mask_img)
        img_size = imA_gray.size
        whitesize = cv2.countNonZero(mask_img)
        if whitesize/img_size < 0.9:
            # ノイズ除去
            mask_img = cv2.medianBlur(mask_img, ksize=15)
            # 白色膨張
            kernel = np.ones((5,5),np.uint8)
            mask_img = cv2.dilate(mask_img, kernel, iterations=3)
            # cv2.imwrite("images/output/" + str(count) + "_mask.jpg", mask_img)
            imA = cv2.inpaint(imB, mask_img, 3, cv2.INPAINT_TELEA)
            # 平滑化
            # imA = cv2.blur(imA, (10, 10))
    new_img = cv2.hconcat([imB, imA])
    cv2.imwrite("images/output/" + str(count) + ".jpg", new_img)
    count += 1


# https://qiita.com/hitomatagi/items/175c7f3a475cf463241b
# http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_photo/py_inpainting/py_inpainting.html
