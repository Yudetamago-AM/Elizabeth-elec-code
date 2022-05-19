#include <Arduino.h>
/*以下lib内自作ライブラリ*/
#include <PinAssign.h>
#include <SD_RW.h>

void setup() {
    /*serial (for debug) initialize*/
    Serial.begin(9600);
    Serial.println(F("Serial begin"));

    sd_init();

    for (byte i = 0; i < 5; i++) {
        //書き込み
        sd_log("hello sd card!>" + String(i));
        delay(1000);
    }

    Serial.println("done!");
}

void loop(){
}