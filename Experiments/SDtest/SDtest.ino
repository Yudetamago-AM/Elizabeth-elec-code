#include <Arduino.h>
/*以下lib内自作ライブラリ*/
#include <PinAssign.h>
#include <SD_RW.h>

void setup() {
    /*serial (for debug) initialize*/
    Serial.begin(9600);
    Serial.println(F("Serial begin"));

    /*SD initialize*/
    while (!SD.begin(PIN_SD_CS)){
        Serial.println(F("SD not ready"));
        delay(100);
    }
    sd_init();//順序間違えない


    for (byte i = 0; i < 5; i++) {
        //書き込み
        sd_log("hello sd card!>" + String(i));
        delay(1000);
    }
}

void loop(){
}