```flow

start=>start: 開始
end=>end: 終了
getgps=>operation: ゴールへの方位計算
getrad=>operation: 機体の方位取得
rotate=>operation: モーター制御
log=>operation: ログ書き込み
isdone=>condition: ゴールまでの
距離 < 3m

start->getgps->getrad->rotate->log->isdone
isdone(yes)->end
isdone(no)->getgps

```