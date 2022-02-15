## FlaskでWebページ作成

### 初回のみ
```
$ pipenv --python 3
$ pipenv install flask
$ pipenv install opencv-python
$ pipenv install numpy
```

### 実行方法
```
$ roscore
```
```
$ python web_face_node.py
```
```
$ rostopic pub -1 /input std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data:
- <int>
- <int>"
```
以下の3つのページにアクセス<br>
http://localhost:3000/<br>
http://localhost:5000/<br>
http://localhost:8000/<br>
