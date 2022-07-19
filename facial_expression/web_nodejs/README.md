## Webページによる眉毛表現

※ラズパイでサーバーを立ち上げると重たいのでノートPCで立ち上げてラズパイ等のデバイスでwebページを開く仕組みにしています。ラズパイでサーバーも立てる場合は[raspi-3bブランチのREADME](https://github.com/MiyabiTane/Deco_with_robot/tree/raspi-3b/facial_expression/web_nodejs)参照

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
$ sudo cp app/app.js src/app.js
```
```
$ docker-compose up
```
して
http://localhost:3000/rbrow, http://localhost:3000/lbrow
にアクセスできればOK

### スマホ等、Webが繋がるデバイスに表示する（最新）
0. ノートPCとデバイスを同じネットワークに繋いでおく

1. ノートPCで以下コマンドを実行してIPアドレスを調べる
    ```
    $ ifconfig
    ```
    `wlp1s0:`の欄の`inet`の後ろの数字が<IPアドレス>

2. ノートPCで眉毛のサーバーを立ち上げる
    ```
    $ cd ver_13types
    $ python run.py  # 既にroscoreが立ち上がっている状況下では`--no-roscore`オプションを追加する
    ```

3. デバイスでWebページを開き、以下ページにアクセスする。横長向きにして全画面表示することをオススメする。<br>
    現状、Google Chromeで安定的に動作する<br>
    左眉毛▷http://<IPアドレス>:3000/lbrow<br>
    右眉毛▷http://<IPアドレス>:3000/rbrow<br>

4. ノートPCから以下コマンドで`/eyebrows/input_type`トピックに0~12の数字を送信して眉毛の動きを変えることができる。
    ```
    $ rostopic pub -1 /eyebrows/input_type std_msgs/Int32 "data: <num>"
    ```
    ```
    Happy😀▷'yorokobi': ['嬉しい']
    Relieved😌▷'yasu': ['安心']
    Astonished😲▷'odoroki': ['びっくり']
    Cry😭▷'aware': ['悲しい']
    Angry😠▷'ikari': ['怒る']
    Flushed😳▷'haji': ['恥ずかしい']
    Fearful😱▷'kowa': ['怖い']
    Love😍▷'suki': ['好き']
    unpleasant😓▷'iya': ['嫌い']
    Smirking😏▷感情対応なし
    Squinting😝▷感情対応なし
    Boring😪▷感情対応なし
    ```

### ラズパイと実機を使った動かし方
0. ノートPCとラズパイを133系ネットワークに繋いでおく

1. [ラズパイで自動画面表示](#ラズパイで自動画面表示)に従ってラズパイを設定しておく

2. ノートPCに[eternal-byte-236613-4bc6962824d1.json](https://drive.google.com/file/d/1VxniytpH9J12ii9jphtBylydY1_k5nXf/view)と[apikey.json](https://drive.google.com/file/d/1wh1_WX3l_qKbUG5wdgeQQBQCu6f9BSWF/view?usp=sharing)をダウンロードする。

3. ノートPCでプログラム起動
    ```
    $ rossetmaster pr1040
    $ rossetip
    $ python run.py --with-chat --nlp-path ${HOME}/Downloads/eternal-byte-236613-4bc6962824d1.json --chat-path ${HOME}/Downloads/apikey.json
    ```
    別端末で
    ```
    $ docker-compose up
    ```

4. ラズパイ上の画面を開く
    1. ノートPCのIPアドレスを調べる
    ```
    $ ifconfig
    # 133系を探す
    ```
    2. 以下コマンドを打つ
    ```
    $ rossetmaster pr1040
    $ rossetip
    $ rostopic pub -1 /facial_expression/restart std_msgs/String data: ''
    # 数秒待ってから
    $ rostopic pub -1 /facial_expression/ip_info std_msgs/String data: <IPアドレス>
    ```

    3. 全画面表示が見切れていたらラズパイにキーボードを繋いで`shift + F5`でリロードする。ページを終了させる場合は`Alt + F4`


### よりシンプルな動かし方
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

#### 初期設定
1. [READMEのRaspberry pi 3B Ubuntu18.04セットアップ](https://github.com/MiyabiTane/Deco_with_robot/tree/raspi-3b/facial_expression/web_nodejs)を参照してUbuntu18.04上で`rossetmaster`コマンドが使える状態のRaspberry pi 3Bを用意する

2. 自動ログイン設定する
    左上のメニューバーから
    `システム管理▷ログイン画面▷Users▷Automatic login▷Username記入`

#### Chromiumをいれる
0. Firefoxでやっても良いが、現状kioskコマンドがうまく動作しなかった。Firefoxを使う場合は、アプリを開いてアドレスバーに`about:config`を入力。`webgl.force-enable`の項目を`true`にしておく

1. インストール
    ```
    $ sudo apt-get update
    $ sudo apt-get upgrade -y
    $ sudo apt-get install chromium-browser
    ```
2. パスワードを求められないようにする
    1. 左上のメニューアイコンで"Passwords and Keys"を検索
    2. 「パスワード」項目下のデフォルトのキーリングを右クリック▷パスワードの変更
    3. 元のパスワードを入力し、パスワードを変更。新しいパスワードは空白にする


#### ユーザー権限のsystemd
0. Chromiumをroot権限から開くのが難しそうだったのでユーザー権限のsystemdを使う
1. `open-browser.py`をホームディレクトリ下に置く
    ```
    $ cp /raspi/open-browser.py ${HOME}/open-browser.py
    ```
2. systemdの設定
    左眉毛の場合はopen-browser-r.serviceをopen-browser-l.serviceに置き換える
    ```
    $ cd ~/.config/systemd/user/
    # ディレクトリが存在しなかったら作る
    $ mkdir -p ~/.config/systemd/user
    $ cp /raspi/open-browser-r.service ~/.config/systemd/user/open-browser-r.service
    $ systemctl --user enable open-browser-r.service
    $ systemctl --user daemon-reload
    $ sudo loginctl enable-linger <username>  # サーバー起動時に自動起動するようにする
    ```

    動作の確認
    ```
    $ sudo reboot now  # 再起動
    $ systemctl --user status open-browser-r.service
    ```
    丸が緑になっていてActiveになっていればOK


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
[ユーザー権限でsystemd](https://pyopyopyo.hatenablog.com/entry/2021/04/30/233755)<br>
[Chromeがパスワードを求めないようにする](http://linuxlabo.labo.main.jp/?eid=4)<br>
[WebGLで複数物体表示](https://www.programmingmat.jp/webgl_lab/triangles.html)<br>
[WebGL多角形の書き方](https://qiita.com/ienaga/items/d9b92d6722aee6465d6c)<br>
