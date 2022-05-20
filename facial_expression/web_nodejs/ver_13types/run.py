import subprocess
import threading
import argparse

def run_roscore():
    subprocess.call(["roscore"])

def run_server():
    subprocess.call(["docker-compose", "up"])

def run_script():
    subprocess.call(["python", "topic_to_eyebrows.py"])

parser = argparse.ArgumentParser()
parser.add_argument("--use-robot", action="store_true")
args = parser.parse_args()

thread_1 = threading.Thread(target=run_roscore)
thread_2 = threading.Thread(target=run_server)
thread_3 = threading.Thread(target=run_script)
if args.use_robot:
    thread_2.start()
    thread_3.start()
else:
    thread_1.start()
    thread_2.start()
    thread_3.start()
