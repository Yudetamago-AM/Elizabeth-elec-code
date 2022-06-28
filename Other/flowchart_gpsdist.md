```flow

start=>start: 開始
end=>end: 終了
backward=>subroutine: 機体の反転検出・修正
rotate90=>operation: 90度左旋回
rotate=>operation: 10度右旋回
checkdist=>inputoutput: 前方の距離確認
ischecked=>condition: 物体検知したか
rotatedir=>operation: 物体の方向へ旋回
forward=>operation: モーター制御
（直進）
log=>operation: ログ書き込み
isgoal=>operation: フェーズ=ゴール判定フェーズ

start->backward->rotate90->rotate->checkdist->ischecked
ischecked(yes)->rotatedir->forward->log->isgoal->end
ischecked(no)->rotate

```