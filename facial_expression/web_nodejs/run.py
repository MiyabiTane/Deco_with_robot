import subprocess
import threading

def run_roscore():
    subprocess.call(["roscore"])

def run_script():
    subprocess.call(["python", "facial_exp_node.py"])

def run_server():
    subprocess.call(["docker-compose", "up"])


thread_1 = threading.Thread(target=run_roscore)
thread_2 = threading.Thread(target=run_server)
thread_3 = threading.Thread(target=run_script)
thread_1.start()
thread_2.start()
thread_3.start()
