### 会話の例
**初めて指示する時**<br>
* もっと右に置いてほしいな
* 風船を少し下にして
* その風船はもっとこっちに置いた方がいいんじゃない？
* それはそっち側がいいと思う
* ピンクの風船はもう少し上に置くと良いと思う
* そこは何も置かない方が良いと思う
* 風船はあそこに置いた方が良いと思う

**二回目以降**<br>
* 良いと思う
* 行き過ぎ
* 元に戻して
* もう少し左
* もっと
* あと少し
* いや、あっち
* そっちじゃなくてこっち
* もっと上にしてほしい

### 会話に含まれる要素
**初めて指示する時**
```
ambiguous_direction = "そっち" | "あっち" | "こっち"
ambiguous_place = "そこ" | "ここ" | "あそこ"
clear_direction = "右" | "左" | "上" | "下"
degree = "もっと" | "少し" | "ちょっと"
indicator = "その" | "あの" | "この"
ambiguous_object = "それ" | "これ" | "あれ"
clear_object = "飾り" | "風船"
denial = "ない" | "違う"
suggestion = "方がいい" | "良さそう" | "てほしい"
action = "置く" | "動かす" | "ずらす" | "する"
```

**二回目以降**
```
excess = "すぎ" | "過ぎ"
denial = "ない" | "違う"
clear_direction = "右" | "左" | "上" | "下"
degree = "もっと" | "少し" | "ちょっと"
action = "置く" | "動かす" | "ずらす" | "する"
denial_action = "戻す"
agree = "良い" | "はい" | "うん"
ambiguous_direction = "そっち" | "あっち" | "こっち"
ambiguous_place = "そこ" | "ここ" | "あそこ"
```


### それをカバーするBNF（旧）
```
<direction> ::= <ambiguous_direction> |
                <clear_direction> |
                <degree><ambiguous_direction> |
                <degree><clear_direction>
<ambiguous_direction> ::= "そっち" | "あっち" | "こっち"
<ambiguous_place> ::= "そこ" | "ここ" | "あそこ"
<clear_direction> ::= "右" | "左" | "上" | "下"
<degree> ::= "もっと" | "少し" | "ちょっと"
<object> ::= <ambiguous_object> |
             <clear_object> |
             <indicator><<ambiguous_object> |
             <indicator><clear_object>
<indicator> ::= "その" | "あの" | "この"
<ambiguous_object> ::= "それ" | "これ" | "あれ"
<clear_object> ::= "飾り" | "風船"
<denial> ::= "ない" | "違う"
<suggestion> ::= "方がいい" | "良さそう" | "てほしい"
<action> ::= "置く" | "動かす" | "ずらす"
```
