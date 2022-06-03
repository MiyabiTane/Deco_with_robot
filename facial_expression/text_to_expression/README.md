## 絵文字に基づいた13眉毛表現

### 使い方
1. 以下のコマンドを実行してJSONファイルを生成する<br>※実行の過程で[youtube-comment-downloader](https://github.com/egbertbouman/youtube-comment-downloader)を利用した
    ```
    $ pipenv run python text_classifier.py
    ```

2. `results`フォルダに生成されたjsonファイルを[Dialogflow](https://dialogflow.cloud.google.com/?authuser=1#/agent/facialexpression-rpwe/intents)のインテントにアップロードして登録する<br>以下の12感情が生成されたことを確認する(Normal表情はどれにも当てはまらない場合の表情なので登録しない)<br>Happy, Relived, Smirking, Astonished, Cry, Angry, Flushed, Fearful, Love, Squinting, Boring, Cold_Sweat

3. ToDo Dialogflowでの実行

### ソース
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
