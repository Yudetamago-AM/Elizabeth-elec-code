```flow

start=>start: 開始
end=>end: 終了
flightPin=>condition: フライトピン解放
からの時間 > 40(s)
timer=>condition: 電源投入からの
時間 > 600(s)
cutting=>subroutine: テグス溶断
log=>operation: ログ書き込み
phase=>operation: フェーズ=GPS誘導フェーズ

start->flightPin(yes)->cutting
flightPin(no)->timer
timer(no)->flightPin()
timer(yes)->cutting->log->phase->end

```
