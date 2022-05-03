/*
ゴールであるカラーコーンの座標を取得してシリアルに出力する．
これで取得した緯度経度などを，Main.inoのほうでつかう．
ほとんどライブラリのサンプルコード
*/

#include <Arduino.h>
#include <Wire.h>
#include <bcdtime.h>
#include <GPS_MTK333X_I2C.h>

GPS_MTK333X_I2C GPS;

void setup() {
    Serial.begin(9600);
    Serial.println(F("serial begin"));

    while (!GPS.begin(9600)) {
        Serial.println(F("GPS not ready"));
        delay(100);
    }

    GPS.sendMTKcommand(314, F(",0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"));
}

void loop() {
    if (GPS.check() && GPS.isTimeUpdate()) {
        bool f = GPS.isStatusUpdate();
        GPSInfo_t gpsInfo = GPS.value();
        Serial.println("gpsInfo.date(BCD date):");
        Serial.print(gpsInfo.date, HEX);
        Serial.write(' ');
        Serial.println("gpsInfo.date(BCD time):");
        Serial.print(gpsInfo.time, HEX);
        Serial.write(' ');
        if (f) {
            Serial.println("gpsInfo.satellites(how many):");
            Serial.print(gpsInfo.satellites, DEC);
            Serial.write(' ');
            Serial.println("gpsInfo.dop(bigger means accurate)(DOP):");
            Serial.print(gpsInfo.dop / 100.0);
            Serial.write(' ');
            Serial.println("gpsInfo.latitude(ido)(degree):");
            Serial.print(gpsInfo.latitude / 600000.0, 6);
            Serial.write(' ');
            Serial.println("gpsInfo.longtitude(keido)(degree):");
            Serial.print(gpsInfo.longitude / 600000.0, 6);
            Serial.write(' ');
            Serial.println("gpsInfo.altitude(koudo)(m):");
            Serial.print(gpsInfo.altitude / 100.0);
            Serial.write(' ');
        }
        Serial.println();
    }
}
