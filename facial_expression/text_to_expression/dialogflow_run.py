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

def run_dummy():
    print("dummy run")

parser = argparse.ArgumentParser()
parser.add_argument("--json-path", default="/home/tork/ros/json/facialexpressionoriginal-cphs-377bf1229657.json")
parser.add_argument("--project-id", default="facialexpressionoriginal-cphs")
parser.add_argument("--no-sample", action="store_true")
args = parser.parse_args()

thread_1 = threading.Thread(target=run_launch, args=(args.json_path, args.project_id,))
thread_2 = threading.Thread(target=run_sub_script)
if args.no_sample:
    thread_3 = threading.Thread(target=run_dummy)
else:
    thread_3 = threading.Thread(target=run_pub_script)
thread_1.start()
thread_2.start()
time.sleep(5)
thread_3.start()
