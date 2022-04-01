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
0. アクセスするサイトは [右眉毛](http://localhost:3000/rbrow), [左眉毛](http://localhost:3000/lbrow)

1. 数値を直接送信して画面の動きを変える
    ```
    $ pyhton run.py --no-sound
    ```
    ```
    $ rostopic pub -1 /degree std_msgs/Float64 "data: 20.0"
    ```

    `data: `以下に任意の数字を指定すると画面の中の動きが変わる

2. 実機の音声データを利用する
    1. `pipenv`が使える環境にしておく

    2. [eternal-byte-236613-4bc6962824d1.json](https://drive.google.com/file/d/1VxniytpH9J12ii9jphtBylydY1_k5nXf/view)をダウンロードする。

    3. 各端末で以下のコマンドを実行（端末を分けているのはログを見やすくするため）
        ```
        $ python run.py --nlp-path ${HOME}/Downloads/eternal-byte-236613-4bc6962824d1.json
        ```
        ```
        $ docker-compose up
        ```

    ※`/robotsound_jp`トピックにデータがpublishされることを想定している

3. 実機と雑談する
    1. [apikey.json](https://drive.google.com/file/d/1wh1_WX3l_qKbUG5wdgeQQBQCu6f9BSWF/view?usp=sharing)をダウンロードする

    2. 各端末で以下のコマンドを実行
        ```
        $ python run.py --with-chat --nlp-path ${HOME}/Downloads/eternal-byte-236613-4bc6962824d1.json --chat-path ${HOME}/Downloads/apikey.json
        ```
        ```
        $ docker-compose up
        ```
    3. お話しする！コツはマイクに向かって話すこと...

### ラズパイからサイトにアクセスする

1. firefoxを開いてアドレスバーに`about:config`を入力。`webgl.force-enable`の項目を`true`にする。これをしないと画面が真っ白になってしまう

2. PCとラズパイを同じネットワークに繋いでおく

3. webを立ち上げているマシン(ノートPC)で`ifconfig`してIPアドレスを調べる

4. ラズパイのfirefoxでhttp://<IPアドレス>:3000/rbrow、IPアドレス>:3000/lbrowにアクセスする

### ラズパイでサーバーを立ち上げる

1. [LOVOTディレクトリのREADME](https://github.com/MiyabiTane/LOVOT)を参照してUbuntu,ROSを入れた状態のラズパイ(3B)を用意する
    1. Ubuntu18のイメージは[こちら](https://drive.google.com/file/d/1f7Y_gSQFexneSPZxi8n0niOyk-UK_Huh/view?usp=sharing)
    2. ディスプレイがつかない時：HDMIをケーブルを先に差し込んでから電源を入れ直してみる
    3. ROSパッケージが更新できなくなった時：以下のコマンドを実行
        ```
        $ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
        ```
    4. SSH出来ない時：以下のコマンドを実行
        ```
        $ sudo rm /etc/ssh/ssh_host_*
        $ sudo dpkg-reconfigure openssh-server
        ```

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

### [おまけ]ラズパイ4セットアップ
1. [公式ページ](https://www.raspberrypi.com/software/)からアプリをダウンロード

2. CHOOSE OSで`Other general purpose OS` ▷ `Ubuntu Desktop`を選択

3. CHOOSE SD CARDで`:/D`を選択

4. ラズパイに差し込んで立ち上げ（HDMI▷電源の順にする）


### 参考記事
[DockerでExpress](https://ishida-it.com/blog/post/2019-11-21-docker-nodejs/)<br>
[jadeの書き方](http://kfug.jp/handson/try_jade/)<br>
[WebGLチュートリアル](https://developer.mozilla.org/ja/docs/Web/API/WebGL_API/Tutorial/Getting_started_with_WebGL)<br>
[HTMLからGETリクエスト](https://stackoverflow.com/questions/6375461/get-html-code-using-javascript-with-a-url)<br>