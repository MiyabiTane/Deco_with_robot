## DockerでWebページ作成

### 初回設定：Webサーバーを立ち上げるまで
```
$ docker-compose run --rm app /bin/bash
# npx express-generator
# npm install
# exit
$ docker-compose up
```
http://localhost:3000
にアクセスしてExpressの画面が出ることを確認する

### 毎回やること
```
$ docker-compose run --rm app /bin/bash
# apt-get update
# apt-get install vim
```
プログラムの編集はvimを使ってDocker内部でやる<br>
本ディレクトリのappフォルダ内のファイルを差し替えor追加する<br>
(ToDo:ローカルの作業をコピー)
```
# exit
$ docker-compose up
```

### 参考記事
[DockerでExpress](https://ishida-it.com/blog/post/2019-11-21-docker-nodejs/)<br>
[jadeの書き方](http://kfug.jp/handson/try_jade/)
[WebGLチュートリアル](https://developer.mozilla.org/ja/docs/Web/API/WebGL_API/Tutorial/Getting_started_with_WebGL)