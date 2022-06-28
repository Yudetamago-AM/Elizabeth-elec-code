```flow

start=>start: 開始
end=>end: 終了
wait=>subroutine: 開傘まで待機
landing=>subroutine: 着地時動作
gpsGuide=>subroutine: GPS誘導
distGuide=>subroutine: 精密誘導
isGoal=>condition: ゴール判定
timesCalled=>condition: 呼出回数 < 3
 
start->wait->landing->gpsGuide->timesCalled

​```
