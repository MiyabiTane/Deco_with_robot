## ロボットが飾り付けを考える

### 初めに

学習結果を利用するだけの場合は[学習結果の利用](#学習結果の利用)の章のみ参を照して下さい。

input_pdfとtraining_checkpointsは中身が重たいため、Google Driveにのせています。ダウンロードして本ディレクトリ直下に置いてください。

input_pdfは[こちら](https://drive.google.com/drive/folders/1izehNB7GK3bh5nqrKKArY0MMVTDhlQMi?usp=sharing)、training_checkpointsは[こちら](https://drive.google.com/drive/folders/1bMuWcRtnufnZGaP-9U49tIJDyV85F8Hg?usp=sharing)

### 学習用データの作成

1. 飾り付け後の画像と、飾り付け部分を白くしたマスク画像がペアとなるように2枚のpdfを作成し、それぞれtrain_B.pdf, train_A.pdfとして```/input_pdf```下に保存する。

    ※この時、pdfは左端から正方形を切り取るので範囲外に画像を貼らないよう注意する。
    
    ※筆者はGoogle Slideにネットから拾った画像を貼り付けてtrain_B.pdfを作成した後、iPadのGoodNotesでpdfを開き、地道に手書きでマスク画像を作成した。もっと賢い方法ありそう。

2. 以下のコマンドを実行すると```images/train```に学習用画像が生成される。
    ```
    $ python3 ./make_image_from_pdf.py --masked
    ```

### pix2pixを用いた学習

1. Google Driveに以下のように配置する。
    ```
    pix2pix_dir
    |-- pix2pix_deco.ipynb
    `-- dataset
        |-- train
        `-- test
            |-- hoge.jpg
            `-- fuga.jpg
    ```
    ここで、trainは先ほど作成したデータセット```train```ディレクトリをそのまま用いる。testには任意の背景画像を.jpg形式で入れる。枚数は何枚でも良い。

2. pix2pix_deco.ipynbをGoogle Colabで開く。2行目はディレクトリの場所によって書き換える。マイドライブ直下に```pix2pix_dir```を配置した場合は以下のようにする。
    ```python
    # drive中の課題ファイルのあるディレクトリに移動
    %cd /content/drive/MyDrive/pix2pix_dir
    ```

3. ランタイム▷ランタイムのタイプを設定でGPUを設定して、全てのセルを実行する。

4. 学習が完了すると```pix2pix_dir/training_checkpoints```下にファイルが生成される。3つのファイルをダウンロードし、本```pix2pix/training_checkpoints```ディレクトリ下に配置する。
    ```
    training_checkpoints
    |-- checkpoint
    |-- ckpt-8.data-00000-of-00001
    `-- ckpt-8.index
    ```

### 学習結果の利用

1. 任意の背景画像を```test.jpg```という名前で保存し、```pix2pix.py```と同じディレクトリ下に置く。
    
    ※　大きさが正方形でない画像は勝手に正方形に変更されてしまうので注意。

2. 上記と同じディレクトリ下で以下のコマンドを叩き、環境構築する。
    ```
    $ pipenv sync
    ```

3. 以下のコマンドで実行すると飾り付け後の画像```output.jpg```が出力される。
    ```
    $ pipenv run python trained_pix2pix.py --input test.jpg
    ```

### 飾り付け配置を生成する

最新スクリプトはこちら▷[Dockerfile](https://github.com/MiyabiTane/jsk_pr2eus/blob/deco_with_robot/pr2eus_tutorials/scripts/deco_demo/Dockerfile), [think_deco_node.py](https://github.com/MiyabiTane/jsk_pr2eus/blob/deco_with_robot/pr2eus_tutorials/scripts/deco_demo/think_deco_node.py), [think_deco.py](https://github.com/MiyabiTane/jsk_pr2eus/blob/deco_with_robot/pr2eus_tutorials/scripts/deco_demo/think_deco.py)

初回のみ、`Dockerfile`が置いてあるディレクトリで以下のコマンドを実行する必要がある
```
$ docker image build -t deco_tensor .
```

### 仮想環境(pipenv)の作り方

0. Linuxの人は[このサイト](https://qiita.com/sabaku20XX/items/67eb69f006adbbf9c525)を参照してpipenvを使える環境にする。Macの人は```brew install```とかで簡単に入るはず。

1. 環境を作りたいディレクトリ下で以下のコマンドを実行することでpython3.7環境を作ることができる。
```
$ pipenv install --python 3.7
```
これにより、PiPfile, PiPfile.lockが作成される。

2. 使いたいツールを仮想環境内にインストール。以下一例。
```
$ pipenv install tensorflow
$ pipenv install matplotlib
$ pipenv install IPython
```

3. PiPfile, PiPfile.lockをコピペして別ディレクトリ下に置き、以下のコマンドを打てばそのディレクトリ下に同じ環境を再現することができる。
```
$ pipenv sync
```

4. 困った時は
```
$ pipenv --rm
$ pipenv sync
```
どこか中間点でおかしなファイルが生成されている場合が多いので、一旦消してから環境を作り直すと直る場合がある。


