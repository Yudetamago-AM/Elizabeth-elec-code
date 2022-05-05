/*
あらかじめ，
https://github.com/askn37/Futilities
https://github.com/askn37/GPS_MTK333X
https://github.com/adafruit/Adafruit_Sensor
https://github.com/adafruit/Adafruit_BusIO
https://github.com/adafruit/Adafruit_BNO055
をライブラリに読み込んでおくこと．
*/


#include <Arduino.h>
#include <Wire.h>
#include <GPS_MTK333X_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PinAssign.h>
#include <Motor.h>

GPS_MTK333X_I2C GPS;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

static int phase = 0;
/*重要！！タイマー！！ミリ秒単位で指定*/
static int Timer = 600 * 1000; //600秒 は 10分

void setup() {
    /*serial (for debug) initialize*/
    Serial.begin(9600);
    Serial.println(F("serial begin"));

    /*BNO055 initialize*/
    while (!bno.begin()) {
        Serial.println("couldn't detect BNO055");
        delay(50);
    }
    bno.setExtCrystalUse(true);//のっかってるからには使わねば（精度向上）

    /*GPS initialize*/
    while (!GPS.begin(9600)) {
        Serial.println(F("GPS not ready"));
        delay(50);
    }
    GPS.sendMTKcommand(314, F(",0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"));
}

void loop() {
    switch (phase)
    {
    case 0:
        Landing();
        break;
    case 1:
        GuideGPS();
        break;
    case 2:
        GuideDIST();
        break;
    case 3:
        Goal();
    }
}

/*各シークエンス*/
void Landing() {
    if (isLanded()) {
        // 前にうごかしたりパラシュート切り離したり…
    }
}

void GuideGPS() {

}

void GuideDIST() {

}

void Goal() {

}

/*Landing用*/
bool isLanded() {
    bool isLanded = false;
    if (Timer <= millis()) {
        phase = 1;
        isLanded = true;
    } else if (isMoving()) {
        phase = 1;
        isLanded = true;
    } else {
        isLanded = false;
    }
    return isLanded;
}

/*その他*/
bool isMoving() {
    /*
    動いていたらtrue，動いていなかったらfalseを返す
    動いていないということは，すなわちスタックしている，ということではない．
    */
    const int thAccel[3] = {10, 20, 10};// 動いているかどうかの閾値設定(m/s^2)　添字0:x軸, 1:y軸，2:z軸
    bool isMoving = false;
    sensors_event_t accelData; 
    bno.getEvent(&accelData);

    if ((accelData.acceleration.x <= thAccel[0]) && (accelData.acceleration.y <= thAccel[1]) && (accelData.acceleration.z <= thAccel[2])) {
        isMoving = false;
    } else {
        isMoving = true;
    }
    return isMoving;
}

double Distance() {
    /*
    距離(cm)を返す
    一応，九軸センサー内蔵の温度センサーで温度を見て，校正しているが，だめっぽかったら15℃として計算してる
    これは, ambient temperature なのか， sensor temperatureなのか，よくわからん（データーシートにも混在）→実験してみる
    */
    int Duration, temp = bno.getTemp();
    double Distance;
    pinMode(PIN_DIST_TRIG, OUTPUT);
    pinMode(PIN_DIST_ECHO, INPUT);

    digitalWrite(PIN_DIST_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_DIST_TRIG, LOW);

    Duration = pulseIn(PIN_DIST_ECHO, HIGH, 23200);//400cm以上の場合0を返す
    if (Duration > 0) {
        if (temp > 5) {
            Distance = Duration * (0.6 * temp + 331.5) / 20000;
            // D(cm) = T(μS) × 1/2(片道) × 340(m/s) × 100(cm/m) × 1/1000000(μS/S)
            // https://nobita-rx7.hatenablog.com/entry/27884818
            // D = T * 0.00005 * c(m/s) = T / 20000
            // c(近似) = 0.6 * temp + 331.5　
        } else {
            Distance = Duration / 58.8;// 15℃ 1013hpa
            // D = T * 0.017 = T / 58.8
        }
    } else {
        Distance = 400;
    }
    return Distance;
}
