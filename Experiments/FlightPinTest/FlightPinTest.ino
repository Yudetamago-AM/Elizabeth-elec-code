#include <Arduino.h>
#include <PinAssign.h>

void setup() {
    /*serial (for debug) initialize*/
    Serial.begin(9600);
    While(Serial){}//シリアル通信開始まで何もしない
    Serial.println(F("Serial begin"));

    /*flight pin init*/
    pinMode(PIN_FLIGHT, INPUT);
}

void loop() {
    if (PIN_FLIGHT == LOW) Serial.prinln("flight pin connected");
    else if Serial.println("flight pin NOT connected");//機体では，パラシュート解放を意味する．

    delay(1000);//（おおよそ）1秒ごとに確認．
}