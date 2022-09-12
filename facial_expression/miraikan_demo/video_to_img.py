import cv2
import glob
import time
import os

FPS = 30  # 29.97

def split_video(video_path):
    cap = cv2.VideoCapture(video_path)
    file_name = video_path[6:-4]
    pre_time = 0
    rate = 1
    while True:
        ret, img = cap.read()
        if not ret:
            break
        rate += 1
        if rate % FPS == 0:  # 1秒ごとに画像として書き出す
            # print("output: ", file_name + str(rate // FPS))
            cv2.imwrite("output/" + file_name + str(rate // FPS) + ".png", img)
    cap.release()
    cv2.destroyAllWindows()

def main():
    os.makedirs('output', exist_ok=True)
    video_paths = glob.glob("input/*.mp4")
    for video_path in video_paths:
        split_video(video_path)
        
main()
