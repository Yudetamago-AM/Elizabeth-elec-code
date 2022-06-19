```flow

start=>start: 開始
end=>end: 終了
rotate90=>operation: 90度旋回
rotate=>operation: 小旋回
checkdist=>operation: 前方の距離確認
ischecked=>condition: 物体検知？
forward=>operation: 直進
log=>operation: ログ書き込み

start->rotate90->rotate->checkdist->ischecked
ischecked(yes)->forward->log->end
ischecked(no)->rotate

```