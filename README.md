## ロボットと部屋の飾り付けをする

### デモ

[MiyabiTane/jsk_pr2eusのdeco_with_robotブランチ](https://github.com/MiyabiTane/jsk_pr2eus/tree/deco_with_robot/pr2eus_tutorials)

### 飾り付け思考部分

[pix2pixディレクトリ](https://github.com/MiyabiTane/Deco_with_robot/tree/main/pix2pix)参照

### 眉毛デバイス

詳細は以下2つのリポジトリ<br>
Dialogflowを用いてテキストから13種の表情を割り当てる▷[facial_expression/text_to_expression](https://github.com/MiyabiTane/Deco_with_robot/tree/main/facial_expression/text_to_expression)<br>
Webページで眉毛を作る▷[facial_expression/web_nodejs](https://github.com/MiyabiTane/Deco_with_robot/tree/main/facial_expression/web_nodejs)

#### 試し方
ノートPC内で全て試せる簡単な試し方。<br>

0. `dialogflow_task_executive`をビルドしていなければビルドする
  ```
  $ mkdir -p eyebrows_ws/src
  $ cd eyebrows_ws/src
  $ git pull https://github.com/jsk-ros-pkg/jsk_3rdparty.git
  $ cd ../
  $ catkin b dialogflow_task_executive
  $ source ~/eyebrows_ws/devel/setup.bash
  ```

1. それぞれのコマンドを別ターミナルで実行する
  ```
  $ cd facial_expression/text_to_expression
  $ python dialogflow_run.py --no-sample
  ```
  ```
  $ cd facial_expression/web_nodejs/ver_13types
  $ python run.py --no-roscore
  ```
2. 2つのWebページにアクセスする▷ http://localhost:3000/lbrow, http://localhost:3000/rbrow

3. 任意のテキストをPublishする。以下は"嬉しい"を送る例。Webページ上の眉毛の動きが変化する。
  ```
  $ rostopic pub -1 /text std_msgs/String "data: '嬉しい'"
  ```
  ※文章の内容によっては動きが変化しない場合がある。


### 対話部分

Chaplusの雑談APIを使用。

#### 使い方

1. [公式ページ](http://www.chaplus.jp/api)にメールアドレスを登録してAPI Keyを入手する。

2. [chaplus_rosリポジトリ](https://github.com/MiyabiTane/jsk_3rdparty/tree/deco_with_robot/chaplus_ros)をcloneした後、apikey.jsonの値を入手したものに書き換える。
```
{"apikey": "0123456789"}
```

3. launchファイル内に以下を記述する。
```
  <!-- add ChaPlus -->
  <arg name="chatbot_engine" default="Chaplus" />
  <node pkg="chaplus_ros" type="chaplus_ros_custom.py" name="chaplus_ros"
        output="screen" >
    <rosparam subst_value="true">
      chatbot_engine: $(arg chatbot_engine)
    </rosparam>
  </node>
```

```/request```をSubscribeして```/response```をPubrishする。型はどちらも```std_msgs/String```

4. エージェントのタイプや、特定の質問に対する返答を指定したい場合など、APIをカスタマイズしたい場合はcustom.jsonを書き換える。詳細と書き換え方は[雑談応答API公式ページ](https://k-masashi.github.io/chaplus-api-doc/ChatAPI.html)の一番下に載っているサンプルコードを参考にする。



