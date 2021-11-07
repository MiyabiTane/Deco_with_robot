## ロボットと部屋の飾り付けをする

### デモ

[MiyabiTane/jsk_pr2eusのdeco_with_robotブランチ](https://github.com/MiyabiTane/jsk_pr2eus/tree/deco_with_robot/pr2eus_tutorials)

### 飾り付け思考部分

[pix2pixディレクトリ](https://github.com/MiyabiTane/Deco_with_robot/tree/main/pix2pix)参照

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



