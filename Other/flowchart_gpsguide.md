```flow

start=>start: 開始
end=>end: 終了

backward=>subroutine: 機体の反転検出・修正
getgps=>inputoutput: GPS座標取得
getrad=>inputoutput: 機体の方位取得
calcgps=>subroutine: ゴールへの方位・距離計算
rotate=>operation: モーター制御（旋回）
log=>operation: ログ書き込み
isdone=>condition: ゴールまでの
距離 < 3m
phase=>operation: フェーズ=精密誘導フェーズ

start->backward->calcgps()->rotate->log->isdone
isdone(yes, right)->phase->end
isdone(no)->end

```