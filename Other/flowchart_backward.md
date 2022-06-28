```flow

start=>start: 開始
end=>end: 終了

getrad=>operation: 機体のピッチ取得
forward=>operation: モーター制御
（前転）
isbackward=>condition: 機体が
反転しているか

start->getrad->isbackward(no)->end
isbackward(yes)->forward->end

```