## DockerでWebページ作成

※本ページはRaspberry pi 3Bにおけるセットアップ方法です。

### Raspberry pi 3B Ubuntu18.04セットアップ

#### 立ち上げと初期設定
0. Win32DiskImagerのインストール(Windows)。SDカードに書き込みを行うアプリで、[このリンク](https://win32-disk-imager.jp.uptodown.com/windows/download)からダウンロードする。

1. [ubuntu-mate-18.04.2-beta1-desktop-arm64+raspi3-ext4.img](https://drive.google.com/file/d/1f7Y_gSQFexneSPZxi8n0niOyk-UK_Huh/view)をダウンロードする

3. SDカード(microSD64GBを使いました)をPCに接続する。Win32DiskImagerを開き、.imgファイルとデバイスを選択してWriteボタンを押すことでSDカードに書き込みが行われる。

4. 書き込みされたSDカードをRaspberry piに挿入し、HDMI、電源、キーボード、マウスを接続。HDMIケーブルを接続してから電源をいれないとディスプレイが認識されないので注意

5. 設定は基本的に全部「日本語」でOK。自動ログインに設定しておく。設定し損ねて後から自動ログインにしたくなったら以下の手順でできる<br>
`システム管理▷ログイン画面▷Users▷Automatic login▷Username記入`

#### ROSをいれる
1. Raspberry pi上のターミナルで[Ubuntu install of ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)のページに従っていれる。

2. rossetmasterしたいので関連ツールをインストールしてターミナル立ち上げ直す
    ```
    $ sudo apt-get install ros-melodic-jsk-topic-tools
    $ sudo apt-get install ros-melodic-jsk-common
    ```

3. ROS関連のインストールでエラーが出たら以下のコマンドを実行してからもう一度インストールする
    ```
    $ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    ```

    ※Wi-Fiが繋がっている必要がある。遅い場合は有線推奨。

#### PCからSSHできるようにする

1. SSHしたい側（ノートPC）で
    ```
    $ ssh-keygen -t rsa
    ```
    ひたすらEnter
    ```
    $ cd ~/.ssh
    $ cat id_rsa.pub
    ```
    ssh-rsaからhoge@fuga まで（★）をコピー

2.  SSHされたい側（Raspberry pi3）で
    1. ssh keyの登録
        ```
        $ cd .ssh
        ```
        .sshディレクトリが存在しなければ作る
        ```
        $ mkdir .ssh
        ```
        authorized_keysファイルに書き込み
        ```
        vi authorized_keys
        ```
        先程コピーしたPCの鍵（★）をファイルに書き込む。vimを使うなら
        ```
        i
        Esc
        :set paste
        ★をペースト
        Esc
        :wq
        ```
        書き込みが完了したら
        ```
        cat authorized_keys
        ```
        を行い、書き込みができているかを確認する。

    2. Raspberry piのポートの設定変更
        ```
        sudo raspi-config
        ```
        して、SSHの項目をenableに設定する。設定が変更できたらRaspberry piを再起動する。

    3. sshを入れ直しておく
        ```
        $ sudo apt-get --purge remove openssh-server
        $ sudo apt-get install openssh-server
        ```

3. SSHしてみる

    以下、SSHしたい側のPCの名前をtork@ubuntu、Raspberry pi側の名前をpr2-tanemoto2@pr2tanemoto2-desktopとする。PCとRaspberry piを同じネットワークに繋いでおく。有線が確実。それぞれの端末でifconfigすれば今繋がっているネットワークを確認できる。<br>

    ノートPCで以下を実行。
    ```
    tork@ubuntu:~/$ ping pr2tanemoto2-desktop.local  # まずpingが通るか確認
    ```
    ```
    tork@ubuntu:~/$ ssh pr2-tanemoto2@pr2tanemoto2-desktop.local

    # もしできなければ以下を試す
    tork@ubuntu:~/$ ssh pr2-tanemoto2@<ラズパイのIPアドレス>
    ```

### 本プログラムを動かすための設定

#### Dockerをいれる

```
$  curl -fsSL https://get.docker.com -o get-docker.sh
$ sudo sh get-docker.sh
$ sudo apt install docker-compose
```

#### pipenvをいれる
1. インストール
    ```
    $ python3 -m pip install pipenv
    ```
2. 以下コマンドで正常に環境が構築されればOK
    ```
    $ python3 -m pipenv sync
    ```
3. もし失敗したら以下を実行するのが一番早い
    ```
    $ rm Pipefile
    $ rm Pipefile.lock
    $ python3 -m pip pipenv --python 3
    $ python3 -m pipenv install google-cloud-storage
    $ python3 -m pipenv install google-cloud-language
    ```

#### 初回設定：Webサーバーを立ち上げるまで
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

#### 開発時に毎回やること
```
$ docker-compose run --rm app /bin/bash
# apt-get update
# apt-get install vim
```
プログラムの編集はvimを使ってDocker内部でやる(`vi hoge.fuga`)<br>

#### 動かし方：シンプルなプログラム（数値を送信して画面を動かす）
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
        $ rossetmaster <ロボット名>
        $ rossetip
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
        $ rossetmaster <ロボット名>
        $ rossetip
        $ python run.py --with-chat --nlp-path ${HOME}/Downloads/eternal-byte-236613-4bc6962824d1.json --chat-path ${HOME}/Downloads/apikey.json
        ```
        ```
        $ docker-compose up
        ```
    3. お話しする！コツはマイクに向かって話すこと...

#### 複数マシンからサイトにアクセスする

1. firefoxを開いてアドレスバーに`about:config`を入力。`webgl.force-enable`の項目を`true`にする。これをしないと画面が真っ白になってしまう

2. サーバーを立ち上げるラズパイと画面を表示する他のマシンを同じネットワークに繋いでおく

3. 各マシンでhttp://<ラズパイのホスト名>:3000/rbrow、http://<ラズパイのホスト名>:3000:3000/lbrowにアクセスする。<br>
例えばhttp://pr2tanemoto2-desktop.local:3000/rbrow



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