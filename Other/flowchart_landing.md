```flow

start=>start: 開始
end=>end: 終了
flightPin=>condition: フライトピン解放
からの時間 > 50(s)
Nichrome=>operation: ニクロム線溶断
forward=>operation: モーター制御
（前進）
log=>operation: ログ書き込み


start->flightPin(yes)->Nichrome()
flightPin(no)->flightPin()
Nichrome->forward->end

```
