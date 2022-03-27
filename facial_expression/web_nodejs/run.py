import subprocess
import threading
import argparse

def run_roscore():
    subprocess.call(["roscore"])

def run_script():
    subprocess.call(["python", "facial_exp_node.py"])

def run_server():
    subprocess.call(["docker-compose", "up"])

def run_analyzer(json_path):
    subprocess.call(["roslaunch", "ros_google_cloud_language", "demo.launch", "google_cloud_credentials_json:=" + json_path])

def run_analyzer_node():
    subprocess.call(["python", "sentiment_analysis_node.py"])

parser = argparse.ArgumentParser()
parser.add_argument("--path", required=True)
args = parser.parse_args()

# thread_1 = threading.Thread(target=run_roscore)
thread_1 = threading.Thread(target=run_analyzer, args=(args.path,))
thread_2 = threading.Thread(target=run_server)
thread_3 = threading.Thread(target=run_script)
thread_4 = threading.Thread(target=run_analyzer_node)
thread_1.start()
thread_2.start()
thread_3.start()
thread_4.start()
