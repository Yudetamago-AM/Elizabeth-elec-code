```flow

start=>start: 処理開始
end=>end: 処理終了
wait=>condition: 開傘したか
landing=>subroutine: 着地シークエンス
gpsGuide=>subroutine: GPS誘導
distGuide=>subroutine: 精密誘導
isGoal=>condition: ゴール判定
timesCalled=>condition: 呼出回数 < 3
 
start->wait(yes)->landing->gpsGuide->timesCalled
wait(no)->wait
timesCalled(yes)->distGuide()->isGoal()
timesCalled(no)->isGoal()
isGoal(yes)->end
isGoal(no)->gpsGuide()
​```
