## 絵文字に基づいた13眉毛表現

### 使い方
1. 以下のコマンドを実行してJSONファイルを生成する。同じ階層に`results`フォルダを作成しておく。<br>※実行の過程で[youtube-comment-downloader](https://github.com/egbertbouman/youtube-comment-downloader)を利用した
    ```
    $ pipenv run python text_classifier.py
    ```

2. Google Driveに`Colab Notebook`というフォルダを作成し、そこに生成された`results`フォルダと`re_classifier.ipynb`をアップロードする。同じ階層に`check`フォルダと`new_results`フォルダを作成し、`re_classifier.ipynb`Google Colabで実行する。

3. `new_results`フォルダに生成されたjsonファイルをダウンロードし、[Dialogflow](https://dialogflow.cloud.google.com/?authuser=1#/agent/facialexpression-rpwe/intents)のインテントにアップロードして登録する<br>以下の12感情が生成されたことを確認する(Normal表情はどれにも当てはまらない場合の表情なので登録しない)<br>Happy, Relived, Smirking, Astonished, Cry, Angry, Flushed, Fearful, Love, Squinting, Boring, Cold_Sweat

4. [dialogflowのjsonファイル](https://drive.google.com/file/d/1NFO6SFLls1CN3fH5byAIk1W5ntGoHhyE/view?usp=sharing)をダウンロードし、`<json_path>`の場所に置く<br>以下のコマンドを実行<br>[眉毛デバイス](https://github.com/MiyabiTane/Deco_with_robot/tree/main/facial_expression/web_nodejs)があればそこにトピックが送られる
    ```
    $ python dialogflow_run.py --json-path <json_path>
    ```
    ※ただし、[dialogflow_task_executive](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/dialogflow_task_executive)がbuildされていて実行できることを前提とする

5. さらに別の例文を試したい場合は以下のようにして試すことができる
    ```
    $ rostopic pub -1 /text std_msgs/String "data: '<words like 感動しました>'"
    ```


### ソース

### Youtubeのコメント欄

Youtubeのコメント欄にて、絵文字を含む文章を収集<br>
[2021年度総視聴回数ランキング](https://webtan.impress.co.jp/n/2022/02/21/42365)を参考に使用するチャンネルを決めました。
1位のチャンネルはショート動画がほとんど、2位のチャンネルはゲーム実況メインであったため、3位のフィッシャーズと、人々の共感を呼びやすい音楽ジャンルを代表して10位のあいみょんの動画を利用した<br>
2022年6月3日現在にて視聴数上位の以下の10チャンネルから、1動画上限5万件、最新順でコメントを収集した<br>
[【MV】虹/Fischer's](https://www.youtube.com/watch?v=0xI4J9CwMuY)<br>
[日本一！？死ぬほど危険すぎて誓約書を書く動物園が怖すぎる！！](https://www.youtube.com/watch?v=Na_WJPK26Oc)<br> 
[笑ってはいけないアニ文字がツボに入ってしまった。](https://www.youtube.com/watch?v=28jAR_LDNJE)<br>
[キレイすぎる海上アスレチックパークの難易度がガチめに高すぎる！？](https://www.youtube.com/watch?v=uxk_qap7pwA)<br> 
[ヘリウムガス吸って絶対言わないセリフをいとこ、兄貴に言わせてやる！！](https://www.youtube.com/watch?v=VadBq-_234g)<br>
[あいみょん - マリーゴールド【OFFICIAL MUSIC VIDEO】](https://www.youtube.com/watch?v=0xSiBpUdW4E)<br>
[あいみょん - 裸の心【OFFICIAL MUSIC VIDEO】](https://www.youtube.com/watch?v=yOAwvRmVIyo)<br>
[あいみょん – ハルノヒ【OFFICIAL MUSIC VIDEO】](https://www.youtube.com/watch?v=pfGI91CFtRg)<br>
[あいみょん - 愛を伝えたいだとか 【OFFICIAL MUSIC VIDEO】](https://www.youtube.com/watch?v=9qRCARM_LfE)<br>
[あいみょん - 君はロックを聴かない 【OFFICIAL MUSIC VIDEO】](https://www.youtube.com/watch?v=ARwVe1MYAUA)<br>

### 感情モデル

Youtubeのコメント欄にはわざとチグハグ表現を使って挑発しているものもあるため、[MLAsk](http://arakilab.media.eng.hokudai.ac.jp/~ptaszynski/repository/mlask.htm)（参考:[Oiita](https://qiita.com/konitech913/items/317b452cc6c63894fce3)）モデルを用いて文章分類を補正している。<br>

さらに、MLAskのソースとなっているemotions.zipをダウンロードし、それらのデータを追加した。表情モデルと感情の対応は以下の通り<br>
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
Smirking😏▷
Squinting😝▷
Boring😪▷

対応なし▷'takaburi': ['興奮']
```
