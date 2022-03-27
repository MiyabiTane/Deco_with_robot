## DockerでWebページ作成

### 初回設定：Webサーバーを立ち上げるまで
```
$ docker-compose run --rm app /bin/bash
# npx express-generator
# npm install
# exit
$ docker-compose up
```
本ディレクトリのappフォルダ内のファイルでDockerの/app以下のファイルを差し替えor追加する<br>
http://localhost:3000
にアクセスしてHello Worldの画面が出ることを確認する

### 開発時に毎回やること
```
$ docker-compose run --rm app /bin/bash
# apt-get update
# apt-get install vim
```
プログラムの編集はvimを使ってDocker内部でやる(`vi hoge.fuga`)<br>

### 動かし方：シンプルなプログラム（数値を送信して画面を動かす）
```
$ pyhton run.py
```
別端末で
```
$ rostopic pub -1 /input std_msgs/Float64 "data: 20.0"
```
http://localhost:3000/rbrow、
http://localhost:3000/lbrow、
http://localhost:3000/mouth
にアクセスする<br>
`data: `以下に任意の数字を指定すると画面の中の動きが変わる

### 動かし方：実機の音声データを利用したプログラム

1. [ros_google_cloud_language](https://github.com/k-okada/jsk_3rdparty/tree/google_nlp/ros_google_cloud_language)が使える環境を作る。

    1. 初回セットアップ
    ```
    $ git clone https://github.com/k-okada/jsk_3rdparty.git
    $ git fetch origin
    $ git checkout google_nlp
    $ catkin build ros_google_cloud_language
    ```

    2. demo.launchを書き換える
    ```
    <launch>
        <arg name="google_cloud_credentials_json" default="" />
    <include file="$(find ros_google_cloud_language)/launch/analyze_text.launch" >
        <arg name="google_cloud_credentials_json" value="$(arg google_cloud_credentials_json)" />
    </include>
    <!-- node pkg="ros_google_cloud_language" type="client.l"
            name="client" output="screen" / -->
    </launch>
    ```

    3. [eternal-byte-236613-4bc6962824d1.json](https://drive.google.com/file/d/1VxniytpH9J12ii9jphtBylydY1_k5nXf/view)をダウンロードする。

2. ros_google_cloud_languageの環境を作ったワークスペースをソースしてプログラム実行
```
$ source <your_ws>/devel/setup.bash
$ python run.py --path ${HOME}/Downloads/eternal-byte-236613-4bc6962824d1.json
```

### ラズパイからサイトにアクセスする

1. firefoxを開いてアドレスバーに`about:config`を入力。`webgl.force-enable`の項目を`true`にする。これをしないと画面が真っ白になってしまう

2. PCとラズパイを同じネットワークに繋いでおく

3. webを立ち上げているマシン(ノートPC)で`ifconfig`してIPアドレスを調べる

4. ラズパイのfirefoxでhttp://<IPアドレス>:3000/rbrow、IPアドレス>:3000/lbrowにアクセスする

### ラズパイでサーバーを立ち上げる

1. [LOVOTディレクトリのREADME](https://github.com/MiyabiTane/LOVOT)を参照してUbuntu,ROSを入れた状態のラズパイを用意する

2. rossetmasterしたいのでツールをインストールする
```
$ sudo apt-get install ros-melodic-jsk-topic-tools
$ sudo apt-get install ros-melodic-jsk-common
```

もし404エラーが出た場合は以下のコマンドを実行してからsudo apt-get installし直す
```
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

以下を実行して確認
```
$ source /opt/ros/melodic/setup.bash
$ rossetmaster
```

3. dockerインストール
```
$  curl -fsSL https://get.docker.com -o get-docker.sh
$ sudo sh get-docker.sh
$ sudo apt install docker-compose
```

4. docker-composeたちあげ
```
$ sudo docker-compose run --rm app /bin/bash
# npx express-generator
# npm install
# exit
$ sudo docker-compose up
```

### 参考記事
[DockerでExpress](https://ishida-it.com/blog/post/2019-11-21-docker-nodejs/)<br>
[jadeの書き方](http://kfug.jp/handson/try_jade/)<br>
[WebGLチュートリアル](https://developer.mozilla.org/ja/docs/Web/API/WebGL_API/Tutorial/Getting_started_with_WebGL)<br>
[HTMLからGETリクエスト](https://stackoverflow.com/questions/6375461/get-html-code-using-javascript-with-a-url)<br>