#include <Arduino.h>
#include <PinAssign.h>

void setup() {
    /*serial (for debug) initialize*/
    Serial.begin(9600);
    while(Serial){}//シリアル通信開始まで何もしない
    Serial.println(F("Serial begin"));

    /*ニクロム線init*/
    pinMode(PIN_NICHROME, OUTPUT);
    digitalWrite(PIN_NICHROME, LOW);

    Serial.println("NICHROME init done");
    Serial.println("would start in 2 sec...");
    delay(2000);//テストのため．

    /*ニクロム線加熱*/
    Serial.println("start heating");

    digitalWrite(PIN_NICHROME, HIGH);
    delay(1000);//1秒間加熱
    digitalWrite(PIN_NICHROME, LOW);

    Serial.println("done heating");
}

void loop() {
}