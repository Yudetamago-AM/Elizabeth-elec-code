```flow

start=>start: 処理開始
end=>end: 処理終了
flightPin=>condition: フライトピン解放
からの時間 > 50(s)
Nichrome=>operation: ニクロム線溶断



start->flightPin(yes)->Nichrome()
flightPin(no)->flightPin()
Nichrome()->end


```
