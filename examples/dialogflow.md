### 会話の例
* もっと右に置いてほしいな
* 風船を少し下にして
* その飾りをちょっと左に置いて
* その風船はもっとこっちに置いた方がいいんじゃない？
* お花はそっち側がいいと思う
* ピンクの風船はもう少し上に置くと良いと思う
* 旗は上の方に飾ろう
* その風船はもっと明るい色のものに変えた方がいいと思うな
* そこは何も置かない方が良いと思う
* 星の風船はハートに変えた方が良さそう


### それをカバーするBNF
```
<direction> ::= <ambiguous_direction> |
                <clear_direction> |
                <degree><ambiguous_direction> |
                <degree><clear_direction>
<ambiguous_direction> ::= "そっち" | "あっち" | "こっち"
<clear_direction> ::= "右" | "左" | "上" | "下"
<degree> ::= "もっと" | "少し" | "ちょっと"
<decoration> ::= <object>
                 <color> + "の" + <object> |
                 <mood><object> |
                 <shape><object>
<object> ::= "飾り" | "風船" | "旗" | "花"
<denial> ::= "ない" | "違う"
<color> ::= "ピンク" | "赤" | "緑" | "黄色"
<mood> ::= "明るい" | "暗い"
<shape> ::= "丸い" | "しかくい" | "さんかくの" | "星の" | "ハートの"
<suggestion> ::= "方がいい" | "良さそう"
```

