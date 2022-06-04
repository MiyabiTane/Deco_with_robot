#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
import threading
import argparse

def run_launch(json_path):
    subprocess.call(["roslaunch", "dialogflow_sample.launch", "google_cloud_credentials_json:=" + json_path])

def run_sub_script():
    subprocess.call(["python", "text_to_expression.py"])

parser = argparse.ArgumentParser()
parser.add_argument("--json-path", default="/home/tork/ros/json/eternal-byte-236613-4bc6962824d1.json")
args = parser.parse_args()

thread_1 = threading.Thread(target=run_launch, args=(args.json_path,))
thread_2 = threading.Thread(target=run_sub_script)
thread_1.start()
thread_2.start()
