import subprocess
import threading
import argparse

def run_roscore():
    subprocess.call(["roscore"])

def run_script():
    subprocess.call(["python", "facial_exp_node.py"])

def run_server():
    subprocess.call(["docker-compose", "up"])

def run_analyzer(nlp_path):
    subprocess.call(["python", "sentiment_analysis_node.py", "--credentials-path", nlp_path])

def run_chat(chat_path):
    subprocess.call(["python", "chat_node.py", "--chat-path", chat_path])

def run_dummy():
    print("dummy thread")

parser = argparse.ArgumentParser()
parser.add_argument("--no-sound", action="store_true")
parser.add_argument("--with-chat", action="store_true")
parser.add_argument("--nlp-path", default="/home/tork/ros/json/eternal-byte-236613-4bc6962824d1.json")
parser.add_argument("--chat-path", default="/home/tork/ros/json/apikey.json")
parser.add_argument("--ip-address", default="")
args = parser.parse_args()

if args.no_sound:
    thread_1 = threading.Thread(target=run_roscore)
    thread_2 = threading.Thread(target=run_server)
    thread_3 = threading.Thread(target=run_script)
    thread_1.start()
    thread_2.start()
    thread_3.start()
elif args.with_chat:
    # thread_0 = threading.Thread(target=run_roscore)
    thread_1 = threading.Thread(target=run_analyzer, args=(args.nlp_path,))
    thread_2 = threading.Thread(target=run_chat, args=(args.chat_path,))
    # thread_0.start()
    thread_1.start()
    thread_2.start()
else:
    thread_1 = threading.Thread(target=run_analyzer, args=(args.nlp_path,))
    thread_1.start()
