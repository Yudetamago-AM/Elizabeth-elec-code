#include <Arduino.h>
#include <Wire.h>
/*以下lib内自作ライブラリ*/
#include <PinAssign.h>
#include <Motor.h>
#include <SD_RW.h>

byte phase = 0;
/*重要！！電源オンからのタイマー，ミリ秒単位で指定*/
const unsigned long Timer = 600000; //600 * 1000, 600秒 は 10分
/*フライトピン解放からのタイマー*/
const unsigned int flightPinTimer = 10000;//50s//10s
unsigned long flightPinMillis;
unsigned long millisTemp = 0;
unsigned long millisTemp2 = 0;
bool isFirstUnplug = true;


void setup() {
    /*serial (for debug) initialize*/
    Serial.begin(9600);
    Serial.println(F("Serial begin"));

    /*Motor initialize*/
    motor_init();
    motor_stop();

    /*ニクロム線の初期設定*/
    pinMode(PIN_NICHROME, OUTPUT);
    digitalWrite(PIN_NICHROME, LOW);

    /*SD initialize*/
    sd_init();
}

void loop() {
    switch (phase) {
    case 0://パラシュート展開まで待つ
        waitTilUnPlug();
        break;
    case 1:
        Landing();
        break;
    case 2:
        GPS.check();
        delay(10000);
        break;
    }
}

void waitTilUnPlug() {
    //フライトピン
    if ((digitalRead(PIN_FLIGHT) == HIGH) && isFirstUnplug) {
        flightPinMillis = millis();
        isFirstUnplug = false;
        Serial.println(F("pin ms set!"));
        sd_log(F("FlightPin removed!"));
        phase = 1;//landing
    } else if (Timer <= millis()) {
        phase = 1;
    }
    delay(50);
}

void Landing() {
    Serial.println(F("Landing seq begin"));
    sd_log(F("landing seq begin"));
    //if (millis() <= flightPinMillis + flightPinTimer) return;
    if (((flightPinMillis + flightPinTimer) <= millis())) {
       // 前にうごかしたりパラシュート切り離したり…
        /*ニクロム線のカット*/
        Nichrome();
    } else if (Timer <= millis()){
        // 前にうごかしたりパラシュート切り離したり…
        /*ニクロム線のカット*/
        Nichrome();
    }

    /*
    //センサーのキャリブレーションしとく
    byte mag;
    for (byte i = 0; i < 100; i++) {
        bno.getCalibration(NULL, NULL, NULL, &mag);
        delay(10);
        if (mag > 0) break;
    }
    Serial.println(F("done"));
    */
}

void Nichrome(){
        digitalWrite(PIN_NICHROME, HIGH);
        delay(2000);
        motor_forward(255, 255);
        delay(3000);
        digitalWrite(PIN_NICHROME,LOW);
        motor_stop();
        sd_log(F("Nichrome heat done!"));
        phase = 2;//gps
}
