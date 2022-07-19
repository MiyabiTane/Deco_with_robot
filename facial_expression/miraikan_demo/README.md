## ペッパーとこちさんのトークイベント

### 本ディレクトリの環境構築
1. 本ディレクトリをコピーし、以下のようなファイル構成にする
    ```
    -- miraikan_ws
       `-- src
           `-- miraikan_demo
    ```

2. ビルドする
    ```
    $ cd miraikan_ws
    $ catkin build miraikan_demo
    $ source ~/miraikan_ws/devel/setup.bash
    ```

### ペッパーの動き・言葉のデモとの組み合わせ
0. `talk_motion.py`を、最新のもの（@a-ichikuraの[talk_motion.py](https://github.com/a-ichikura/miraikan/blob/master/pepper_talk/talk_motion.py)）で置き換える
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
3. `talk_motion.py`を実行してPepperが動かせることを確認する<br>
    ```
    $ python
    >>> import talk_motion
    >>> talk = talk_motion.Talk()
    >>> talk.episode_11()
    ```

### Pepperのイベントプログラムへの組み込み
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

### 実行方法
1. 以下の図のようにネットワーク接続を行う。<br>
    <img width="400" src="./img_README/network_connection.png"><br>

2. PC2で眉毛デバイスサーバーを立ち上げる。以降、PC2のIPアドレスを`<IP_PC2>`とする。
    ```
    $ cd ../web_nodejs
    $ docker-compose up
    ```
    ※眉毛デバイスサーバーのセットアップは[web_nodejs:初回設定](https://github.com/MiyabiTane/Deco_with_robot/tree/main/facial_expression/web_nodejs#%E5%88%9D%E5%9B%9E%E8%A8%AD%E5%AE%9A)を参照。<br>
    ※IPアドレスはターミナルに`$ifconfig`と打ち込んだ時の`wlp1s0:`下の`inet`以降の値`xxx.xxx.xxx.xxx`

3. スマートフォンを2台用意し、それぞれで左眉毛(http://<IP_PC2>:3000/lbrow)、右眉毛(http://<IP_PC2>:3000/rbrow)の画面を表示する。<br>
    ※ブラウザはGoogle Chrome推奨。ページが立ち上がったらアドレスバーを隠すよう、上にスワイプする。

4.  PC2にジョイスティックを接続して以下を実行
    ```
    $ source ~/miraikan_ws/devel/setup.bash
    $ roslaunch lecture-demo.launch eyebrows_server_ip:="<IP_PC2>"
    ```
    ```
    $ roseus lecture-demo.l
    $ (main)
    ```

5. ジョイスティックのボタンを押してPepperが話して動き、ブラウザ上の眉毛が動けばOK

###  本PCで試す方法
1. 実機に繋いで試す場合はPepperと有線接続し、PCの設定からネットワーク▷有線▷Pepperを選択する。Pepperの電源を入れてIPアドレスを聞き取ったら以下を実行する
    ```
    $ export NAO_IP="<Pepper_IP>"
    ```
    詳細は[ペッパーの動き・言葉のデモとの組み合わせ](#ペッパーの動き・言葉のデモとの組み合わせ)参照

2. launchファイルの立ち上げ
    ```
    $ source ~/miraikan_ws/devel/setup.bash
    $ roslaunch miraikan_demo demo.launch run_eyebrows_server:=true
    # 実機と繋いでいる場合は
    $ roslaunch miraikan_demo demo.launch run_eyebrows_server:=true use_robot:=true
    ```
    ※以下のファイル構成になっている前提。眉毛サーバーを別で立てる場合は`run_eyebrows_server:=false`にする
    ```
    -- miraikan_ws
       `-- src
           |-- miraikan_demo
           `-- web_nodejs
               |-- src
               `--docker-compose.yml
    ```

3. euslispファイルの実行
    ```
    $ source ~/miraikan_ws/devel/setup.bash
    $ roscd miraikan_demo
    $ cd euslisp
    $ roseus demo.l
    $ (main)
    ```

4. コマンドからサービスコールする。
    ```
    $ rosservice call /deai_1 "{}"
    $ rosservice call /deai_2 "{}"
    ```

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

### [旧]ペッパーの動き・言葉のデモとの組み合わせ
[fake_motion_talk.py](https://github.com/MiyabiTane/Deco_with_robot/blob/main/facial_expression/miraikan_demo/scripts/fake_motion_talk.py)を@a-ichikuraの[talk_motion.py](https://github.com/a-ichikura/miraikan/blob/master/pepper_talk/talk_motion.py)で置き換える。この時、talk_motion.pyの`if __name__ == '__main__':`以下はコメントアウトする。
```
if __name__ == '__main__':
    talk = Talk()
    talk.episode_11()
    talk.episode_12()
    talk.episode_13()
```
