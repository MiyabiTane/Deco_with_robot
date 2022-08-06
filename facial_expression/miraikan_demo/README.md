## ペッパーとこちさんのトークイベント

### 本ディレクトリの環境構築
1. クローンとビルド
    ```
    $ mkdir -p miraikan_ws/src
    $ cd miraikan_ws/src
    $ git clone git@github.com:MiyabiTane/Deco_with_robot.git
    $ cd ../
    $ catkin build miraikan_demo
    $ source ~/miraikan_ws/devel/setup.bash
    ```

2. 眉毛デバイスのサーバーを立ち上げる場合には以下も行う<br>

    0. [公式サイト](https://docs.docker.com/engine/install/ubuntu/)を参照してdockerとdocker-composeをインストールしておく。sudoがなくてもdockerが立ち上がるよう、以下のコマンドを実行し、PCを再起動する<br>
        ```
        $ sudo groupadd docker
        $ sudo usermod -aG docker $USER
        ```

    1. Webサーバの環境構築
        ```
        $ source ~/miraikan_ws/devel/setup.bash
        $ roscd miraikan_demo
        $ cd ../web_nodejs
        $ docker-compose run --rm app /bin/bash
        # npx express-generator
        # npm install
        # exit
        $ docker-compose up
        ```
        http://localhost:3000 にアクセスしてHello Worldの画面が出ることを確認する

    2. 眉毛プログラムのDockerへのコピー
        ```
        $ sudo cp app/route/* src/routes/  # 本ディレクトリtypoしてるので注意
        $ sudo cp app/public/javascripts/* src/public/javascripts
        $ sudo cp app/views/* src/views
        $ sudo cp app/app.js src/app.js
        ```
        ```
        $ docker-compose up
        ```
        http://localhost:3000/rbrow, http://localhost:3000/lbrow にアクセスできればOK。Chromeブラウザ推奨。

### ペッパーとの接続・動作確認
0. Pepperのボタン操作<br>
    長押し▷電源ON,OFF<br>
    2回押し▷レストモードON, OFF<br>
    1回押し▷IPアドレス確認<br>

1. Pepperとの接続：初回設定
    1. PepperとPCを有線接続し、設定からIPV4メソッドをリンクローカルのみに設定する。Pepperと名前を付けて設定を保存しておく。<br>
        <img width="400" src="./img_README/setup_naoqi_network.png"><br>
    2. [pynaoqi-python2.7-2.5.5.5-linux64](https://drive.google.com/file/d/1xHuYREDa78xGiikEpsjxfZQ7Gfvo1E9D/view)をダウンロードし、.bashrcに以下を追記する
        ```
        export PYTHONPATH=$HOME/Downloads/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages:$PYTHONPATH
        ```
2. Pepperの起動と接続<br>
    ディスプレイ下の電源ボタンを長押しして起動。起動したらもう一度軽く押す。するとPepperが自分のIPアドレス`<Pepper_IP>`を音声で教えてくれる。`talk_motion.py`15行目の`self.PEPPER_IP=`以降のIPアドレスを`<Pepper_IP>`で書き換える。また、以下コマンドでPATHを通す
    ```
    $ export NAO_IP="<Pepper_IP>"
    ```
3. `talk_motion.py`を実行してPepperが動いて喋ることを確認する<br>
    ```
    $ python
    >>> import episode_motion
    >>> talk = episode_motion.Talk("<Pepper_IP>")
    >>> talk.episode_11()
    ```

### Scratchの環境構築
1. npmのインストール（安易にaptでいれるとROSが消えるので注意する！！）
    ```
    $ sudo snap install node --channel=14/stable --classic
    $ npm --version
    # 6.14.17になることを確認する
    ```

2. scratch-rosの環境構築
    ```
    $ mkdir scratch
    $ cd scratch
    $ git clone https://github.com/Affonso-Gui/scratch3-ros-parser
    $ git clone -b jsk_robots https://github.com/Affonso-Gui/scratch3-ros-vm
    $ git clone -b jsk_robots https://github.com/Affonso-Gui/scratch3-ros-gui
    # 以下、時間がかかっても気長に待つこと
    $ cd scratch/scratch3-ros-vm && npm install
    $ cd ../scratch-ros-parser && npm install
    $ cd ../scratch-ros-gui && npm install
    $ cd ../scratch-ros-vm && sudo npm link
    $ cd ../scratch-ros-parser && sudo npm link
    $ cd ../scratch-ros-gui && sudo npm link
    $ npm link scratch-vm
    $ npm link scratch-parser
    $ npm start
    ```
    別ターミナルで
    ```
    $ roslaunch rosbridge_server rosbridge_websocket.launch
    ```
    http://0.0.0.0:8601/ にアクセスして、画面が立ち上がればOK

3. 本デモ用ボックスの追加<br>

    `scratch3-ros-vm/src/extensions/scratch3_pepperrobot/index.js`を[kochigami/add-pepper-extensionのもの](https://github.com/kochigami/scratch3-ros-vm/blob/add-pepper-extension/src/extensions/scratch3_pepperrobot/index.js)に書き換えて以下を実行
    ```
    $ cd ~/scratch/scratch-gui
    $ npm start
    ```
    別ターミナルで
    ```
    $ roslaunch rosbridge_server rosbridge_websocket.launch
    ```
    http://0.0.0.0:8601/ にアクセスして、左下ボタンからメニューページに飛び、Pepperを選択。Mster URLを尋ねられるので`localhost`と打ち込む。

### 実行方法
1. 以下の図のようにネットワーク接続を行う。<br>
    <img width="350" src="./img_README/network_connection.png"><br>

2. PCの設定からネットワーク▷有線▷Pepperを選択する。Pepperの電源を入れてIPアドレス`<Pepper_IP>`を聞き取ったら以下を実行する
    ```
    $ export NAO_IP="<Pepper_IP>"
    ```
    詳細は[ペッパーとの接続・動作確認](#ペッパーとの接続・動作確認)参照

3. ターミナルに`$ifconfig`と打ち込んだ時の`wlp1s0:`下の`inet`以降の値`<EServer_IP>`を確認する。スマートフォンを2台用意し、それぞれで左眉毛(http://<EServer_IP>:3000/lbrow)、右眉毛(http://<EServer_IP>:3000/rbrow)の画面を表示する。<br>
    ※ブラウザはGoogle Chrome推奨。ページが立ち上がったらアドレスバーを隠すよう、上にスワイプする。

4. scratchの立ち上げ（スクラッチを用いずに[rosservice](#rosservice)をターミナルから直接呼び出すこともできる）
    ```
    $ cd scratch/scratch-gui
    $ npm start
    ```
    ※[Scratchの環境構築](#Scratchの環境構築)が終わっている必要がある

5. launchファイルの立ち上げ
    ```
    $ source ~/miraikan_ws/devel/setup.bash
    $ roslaunch miraikan_demo lecture-demo.launch pepper_ip:="<Pepper_IP>" run_eyebrows_server:=true memories_talk:=<true or false>
    ```
    http://0.0.0.0:8601/ にアクセスして、左下ボタンからメニューページに飛び、Pepperを選択。Mster URLを尋ねられるので`localhost`と打ち込む。デモ用に追加したgreeting以下のボックスを押すと、Pepperと眉毛が動く

6. `lecture-demo.launch`の引数詳細<br>
    `run_eyebrows_server` launchを立ち上げる際に眉毛デバイスサーバーを立ち上げるか否か<br>
    `eyebrows_server_ip` 眉毛デバイスのサーバーをたてているIPアドレス。。眉毛デバイスのサーバーを別のPCで立てている場合は指定が必要。<br>
    `use_robot` 実機を繋いでいるか否か。繋いでいない場合は実機の動作の代わりにターミナルに経過秒数が表示される。<br>
    `pepper_ip` 実機のIPアドレス。use_robotがTrueの場合は指定が必要<br>
    `memories_talk` Trueの場合は思い出語りver, Falseの場合は発表形式verのデモが起動する。use_robotがFalseの場合には機能しない<br>

7. 眉毛の左右ずれの調節<br>
    用いているスマホによっては左右の眉毛の動きがずれる場合がある。その場合は[lbrow.jade](https://github.com/MiyabiTane/Deco_with_robot/blob/main/facial_expression/web_nodejs/app/views/lbrow.jade#L14), [rbrow.jade](https://github.com/MiyabiTane/Deco_with_robot/blob/main/facial_expression/web_nodejs/app/views/rbrow.jade#L14)の`delay_ms`を調節する。<br>
    ファイルを変更したら
    ```
    web_nodejs$ sudo cp app/views/* src/views
    ```
    して変更を反映させること。src以下のファイルをsudoで直接いじっても良い。

### rosservice
1. /demo_mode
    ```
    subscribe: mode       ... int32: 実行するデモ（動き・喋り・眉毛）のモード
             : time_delay ... int32: デモを開始して何秒後に眉毛を動かし始めるか
    publish  : success    ... bool : service callの呼び出しに成功したか否か 
    ```
2. /demo_deai_1, /demo_deai_2
    ```
    Empty service
    ```

※コマンドから呼び出す例
```
$ rosservice call /episode_11 "{}"
$ rosservice call /demo_mode "mode: 1 time_delay: 5"
```

### [旧]ペッパーの動き・言葉のデモとの組み合わせ
[fake_motion_talk.py](https://github.com/MiyabiTane/Deco_with_robot/blob/main/facial_expression/miraikan_demo/scripts/fake_motion_talk.py)を@a-ichikuraの[talk_motion.py](https://github.com/a-ichikura/miraikan/blob/master/pepper_talk/talk_motion.py)で置き換える。この時、talk_motion.pyの`if __name__ == '__main__':`以下はコメントアウトする。
```
if __name__ == '__main__':
    talk = Talk()
    talk.episode_11()
    talk.episode_12()
    talk.episode_13()
```

### [旧]Pepperのイベントプログラムへの組み込み
※以下、@kochigamiの[interactive_robot_demo](https://gitlab.jsk.imi.i.u-tokyo.ac.jp/kochigami/interactive_robot_demo/-/blob/master/lecture_demo)と組み合わせる場合を例として説明する

1. lecture-demo.launchに以下を追加する
    ```
    <arg name="eyebrows_server_ip" default="localhost" />
    <arg name="run_eyebrows_server" default="false" />
    <arg name="use_robot" default="true" />

    <include file="$(find miraikan_demo)/launch/demo.launch">
        <arg name="eyebrows_server_ip" value="$(arg eyebrows_server_ip)"/>
        <arg name="run_eyebrows_server" value="$(arg run_eyebrows_server)" />
        <arg name="use_robot" value="$(arg use_robot)" />
    </include>
    ```

2. 本ディレクトリのdemo.lの名前をrobot-behaior-server.lに変更し、もとあるものと置き換える