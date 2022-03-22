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

### 動かし方
```
$ docker-compose up
```
別端末で
```
$ curl -X POST --data-urlencode 'degree=20' http://localhost:3000/api/info
```
http://localhost:3000/rbrow、
http://localhost:3000/lbrow、
http://localhost:3000/mouth、
にアクセスする<br>
`degree=`以下に任意の数字を指定すると画面の中の動きが変わる


### 参考記事
[DockerでExpress](https://ishida-it.com/blog/post/2019-11-21-docker-nodejs/)<br>
[jadeの書き方](http://kfug.jp/handson/try_jade/)<br>
[WebGLチュートリアル](https://developer.mozilla.org/ja/docs/Web/API/WebGL_API/Tutorial/Getting_started_with_WebGL)<br>
[HTMLからGETリクエスト](https://stackoverflow.com/questions/6375461/get-html-code-using-javascript-with-a-url)<br>