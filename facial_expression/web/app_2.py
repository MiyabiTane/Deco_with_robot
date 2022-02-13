import os
import io
import time
import numpy as np
import cv2
from flask import Flask, render_template, request, redirect, url_for, send_from_directory, session
app = Flask(__name__)

UPLOAD_FOLDER = '/static'
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

@app.route('/')
def index():
    raw_img_url = os.path.join(app.config['UPLOAD_FOLDER'], 'face.jpg')
    return render_template('index.html', raw_img_url=raw_img_url)

if __name__ == '__main__':
    app.debug = True
    app.run(host='localhost', port=3000)
