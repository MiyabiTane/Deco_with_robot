## ペッパーとこちさんのトークイベント

### 本ディレクトリの環境構築
1. 本ディレクトリをコピーし、以下のようなファイル構成にする
    ```
    --- miraikan_ws
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
[fake_motion_talk.py](https://github.com/MiyabiTane/Deco_with_robot/blob/main/facial_expression/miraikan_demo/scripts/fake_motion_talk.py)を@a-ichikuraの[talk_motion.py](https://github.com/a-ichikura/miraikan/blob/master/pepper_talk/talk_motion.py)で置き換える。この時、talk_motion.pyの`if __name__ == '__main__':`以下はコメントアウトする。
```
if __name__ == '__main__':
    talk = Talk()
    talk.episode_11()
    talk.episode_12()
    talk.episode_13()
```

### Pepperのイベントプログラムへの組み込み
※以下、@kochigamiの[interactive_robot_demo](https://gitlab.jsk.imi.i.u-tokyo.ac.jp/kochigami/interactive_robot_demo/-/blob/master/lecture_demo/lecture-demo.launch)と組み合わせる場合を例として説明する

1. lecture-demo.launchに以下を追加する
    ```
    <include file="$(find miraikan_demo)/launch/gazebo/demo.launch">
    </include>
    ```

2. 本ディレクトリのdemo.lの名前をrobot-behaior-server.lに変更し、もとあるものと置き換える

### 実行方法
1. 眉毛デバイスサーバーを立ち上げる。同じネットワークに繋がっている別PCで立ち上げる場合はスキップして良い。詳細は[web_nodejs:初回設定](https://github.com/MiyabiTane/Deco_with_robot/tree/main/facial_expression/web_nodejs#%E5%88%9D%E5%9B%9E%E8%A8%AD%E5%AE%9A)を参照。
    ```
    $ cd ../web_nodejs
    $ docker-compose up
    ```

2.  ジョイスティックをパソコンに接続して以下を実行
    ```
    $ source ~/miraikan_ws/devel/setup.bash
    $ roslaunch lecture-demo.launch
    ```
    ```
    $ roseus lecture-demo.l
    $ (main)
    ```

3. ジョイスティックのボタンを押してPepperが話して動き、ブラウザ上の眉毛が動けばOK


###  本PCで試す方法
0. 眉毛デバイスのサーバーを立ち上げる
    [web_nodejs/README.md:初回設定](https://github.com/MiyabiTane/Deco_with_robot/tree/main/facial_expression/web_nodejs#%E5%88%9D%E5%9B%9E%E8%A8%AD%E5%AE%9A)に従ってサーバーのセットアップを行い、`docker-compose up`する<br>
    http://localhost:3000/lbrow, http://localhost:3000/rbrowにアクセスする

1. launchファイルの立ち上げ
    ```
    $ source ~/miraikan_ws/devel/setup.bash
    $ roslaunch miraikan_demo demo.launch
    ```

2. euslispファイルの実行
    ```
    $ source ~/miraikan_ws/devel/setup.bash
    $ roscd miraikan_demo
    $ cd euslisp
    $ roseus demo.l
    $ (main)
    ```

3. コマンドからサービスコールする。
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
