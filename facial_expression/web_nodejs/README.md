## DockerでWebページ作成

※ラズパイでサーバーを立ち上げると重たいのでノートPCで立ち上げてラズパイでwebページを開く仕組みにしています。ラズパイでサーバーも立てる場合は[raspi-3bブランチのREADME](https://github.com/MiyabiTane/Deco_with_robot/tree/raspi-3b/facial_expression/web_nodejs)参照

### 初回設定

#### Webサーバーを立ち上げるまで
```
$ docker-compose run --rm app /bin/bash
# npx express-generator
# npm install
# exit
$ docker-compose up
```
http://localhost:3000
にアクセスしてHello Worldの画面が出ることを確認する

#### 本ディレクトリのDockerへのコピー
```
$ sudo cp app/route/* src/routes/  # 本ディレクトリtypoしてるので注意
$ sudo cp app/public/javascripts/* src/public/javascripts
$ sudo cp app/views/* src/views
```
```
$ docker-compose up
```
して
http://localhost:3000/rbrow, http://localhost:3000/lbrow
にアクセスできればOK

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

※以下。ノートPCとラズパイはロボットと同じネットワークに繋いでおきます

2. 文章を感情分析する
    1. `pipenv`が使える環境にしておく

    2. [eternal-byte-236613-4bc6962824d1.json](https://drive.google.com/file/d/1VxniytpH9J12ii9jphtBylydY1_k5nXf/view)をダウンロードする。

    3. 各端末で以下のコマンドを実行（端末を分けているのはログを見やすくするため）
        ```
        $ rossetmaster <ロボット名>
        $ rossetip
        $ python run.py --nlp-path ${HOME}/Downloads/eternal-byte-236613-4bc6962824d1.json
        ```
        ```
        $ docker-compose up
        ```

    4. `std_msgs/String`型の`/robotsound_text`ノードに文章をpublishする。


3. 実機と雑談する
    1. [apikey.json](https://drive.google.com/file/d/1wh1_WX3l_qKbUG5wdgeQQBQCu6f9BSWF/view?usp=sharing)をダウンロードする

    2. 各端末で以下のコマンドを実行
        ```
        $ rossetmaster <ロボット名>
        $ rossetip
        $ python run.py --with-chat --nlp-path ${HOME}/Downloads/eternal-byte-236613-4bc6962824d1.json --chat-path ${HOME}/Downloads/apikey.json
        ```
        ```
        $ docker-compose up
        ```
    3. お話しする！コツはマイクに向かって話すこと...


### ラズパイで自動画面表示

1. [READMEのRaspberry pi 3B Ubuntu18.04セットアップ](https://github.com/MiyabiTane/Deco_with_robot/tree/raspi-3b/facial_expression/web_nodejs)を参照してUbuntu18.04上で`rossetmaster`コマンドが使える状態のRaspberry pi 3Bを用意する

2. 自動ログイン設定する
    左上のメニューバーから
    `システム管理▷ログイン画面▷Users▷Automatic login▷Username記入`

3. Chromiumインストール
    ```
    $ sudo apt-get update
    $ sudo apt-get upgrade -y
    $ sudo apt-get install chromium-browser
    ```

4. 起動時の動作設定
    1. `open-browser.py`をホームディレクトリ下に置く
        ```
        $ cp /raspi/open-browser.py ${HOME}/open-browser.py
        ```
    2. systemdの設定
        左眉毛の場合はopen-browser-r.serviceをopen-browser-l.serviceに置き換える
        ```
        $ sudo cp /raspi/open-browser-r.service /etc/systemd/system/open-browser-r.service
        $ sudo systemctl daemon-reload
        $ sudo systemctl enable open-browser-r.service
        ```
        動作の確認
        ```
        $ sudo reboot now  # 再起動
        $ sudo systemctl status open-browser-r.service
        ```
        丸が緑になっていてActiveになっていればOK

5. 以下コマンドを打つとラズパイでweb画面が開く
    ```
    $ rossetmaster
    $ rossetip
    $ rostopic pub -1 /facial_expression/ip_info std_msgs/String data: <IPアドレス>
    ```
    `<IPアドレス>`のところに`ifconfig`して調べたノートPCのIPアドレスをいれる


※firefoxで表示する場合は、firefoxを開いてアドレスバーに`about:config`を入力。`webgl.force-enable`の項目を`true`にしておく

※全画面表示中のブラウザを終了させる場合は`Alt + F4`

### 旧情報

#### 初回設定：Webサーバーを立ち上げるまで（旧）
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

#### 開発時に毎回やること（旧）
```
$ docker-compose run --rm app /bin/bash
# apt-get update
# apt-get install vim
```
プログラムの編集はvimを使ってDocker内部でやる(`vi hoge.fuga`)<br>

### 参考記事
[DockerでExpress](https://ishida-it.com/blog/post/2019-11-21-docker-nodejs/)<br>
[jadeの書き方](http://kfug.jp/handson/try_jade/)<br>
[WebGLチュートリアル](https://developer.mozilla.org/ja/docs/Web/API/WebGL_API/Tutorial/Getting_started_with_WebGL)<br>
[HTMLからGETリクエスト](https://stackoverflow.com/questions/6375461/get-html-code-using-javascript-with-a-url)<br>