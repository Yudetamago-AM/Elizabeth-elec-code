#include <Arduino.h>
#include <PinAssign.h>

void setup() {
    /*serial (for debug) initialize*/
    Serial.begin(9600);
    while(!Serial){}//シリアル通信開始まで何もしない
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
    delay(5000);//5秒間加熱
    /*
    実験してみたところ，ニクロム線が短すぎると，バッテリの短絡防止回路がうごいて，
    モーター・ニクロム線系統の電源が落ちてしまう．
    ニクロム線を長めに取る（5Ω分くらい）と，うまくいく（抵抗を付けるのもありかも→要実験）．
    溶断にかかる時間は，ある程度テンションをかけた状態で，3秒ほどであった．
    このため，ニクロム線の長さにもよるが，5秒ほど電流を流した方が良いかと思う．
    →ReadMeに転記した．
    */
    digitalWrite(PIN_NICHROME, LOW);

    Serial.println("done heating");
}

void loop() {
}
