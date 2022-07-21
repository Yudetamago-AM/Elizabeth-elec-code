```flow

start=>start: 開始
end=>end: 終了
end2=>end: 終了

backward=>subroutine: 機体の反転検出・修正
gpscheck=>operation: ゴールまでの距離・方位取得
dist400=>condition: ゴールまでの
距離 < 400(cm)
goalcount=>condition: ゴール判定の
呼出回数 < 5
distfor=>inputoutput: 機体前側の距離取得
distfor2=>inputoutput: 機体前側の距離取得
rotate=>operation: 小旋回
phasegps=>operation: フェーズ=GPS誘導
phasedist=>operation: フェーズ=精密誘導
phase5=>operation: フェーズ=回収まで待機
phase5b=>operation: フェーズ=回収まで待機
log=>operation: ログ書き込み
log2=>operation: ログ書き込み
check=>condition: 距離 < 12(cm)
calcgps=>subroutine: ゴールへの方位・距離計算

start(right)->backward(right)->calcgps->dist400
dist400(no)->goalcount
dist400(yes, right)->phasegps->phase5->log->end2
goalcount(no)->phasegps
goalcount(yes, right)->distfor->check
check(yes)->log2->phase5b->end
check(no)->phasedist->phase5b

```
