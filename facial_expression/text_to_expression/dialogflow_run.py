#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
import threading
import argparse
import time

def run_launch(json_path, project_id):
    # print(json_path, project_id)
    subprocess.call(["roslaunch", "dialogflow_sample.launch", "google_cloud_credentials_json:=" + json_path,
                     "project_id:=" + project_id])

def run_sub_script():
    subprocess.call(["python", "text_to_expression.py"])

def run_pub_script():
    subprocess.call(["python", "sample_publisher.py"])

parser = argparse.ArgumentParser()
parser.add_argument("--json-path", default="/home/tork/ros/json/facialexpression-rpwe-01533d0109c5.json")
parser.add_argument("--project-id", default="facialexpression-rpwe")
args = parser.parse_args()

thread_1 = threading.Thread(target=run_launch, args=(args.json_path, args.project_id,))
thread_2 = threading.Thread(target=run_sub_script)
thread_3 = threading.Thread(target=run_pub_script)
thread_1.start()
thread_2.start()
time.sleep(5)
thread_3.start()
