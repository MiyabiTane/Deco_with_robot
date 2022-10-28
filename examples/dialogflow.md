### 会話の例
* もっと右に置いてほしいな
* 風船を少し下にして
* その風船はもっとこっちに置いた方がいいんじゃない？
* それはそっち側がいいと思う
* ピンクの風船はもう少し上に置くと良いと思う
* そこは何も置かない方が良いと思う
* 風船はあそこに置いた方が良いと思う

### それをカバーするBNF
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
