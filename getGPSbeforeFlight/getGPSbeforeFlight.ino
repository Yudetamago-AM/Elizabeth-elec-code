/*
ゴールであるカラーコーンの座標を取得してシリアルに出力する．
これで取得した緯度経度などを，Main.inoのほうでつかう．
ほとんどライブラリのサンプルコード
*/

#include <Arduino.h>
#include <Wire.h>
#include <PinAssign.h>
//#include <GPS_MTK333X_Serial.h>
#include <SoftwareSerial.h>
#include <GPS_MTK333X_SoftwareSerial.h>

GPS_MTK333X_SoftwareSerial GPS(PIN_GPS_RX, PIN_GPS_TX);

//GPS_MTK333X_Serial GPS;

void setup() {
    Serial.begin(9600);
    Serial.println(F("serial begin"));

    while (!GPS.begin(9600)) {//9600
        Serial.println(F("GPS not ready"));
        delay(100);
    }

    // UARTボーレートを 9600bpsに設定する
    GPS.sendMTKcommand(251, F(",9600"));
    // 500ms間隔で NMEAを出力する
    GPS.sendMTKcommand(220, F(",500"));//500msごとで十分かは要検証
    // AlwaysLocate モード開始 
    //GPS.sendMTKcommand(225, F(",8"));
    // 上記を解除してスタンダードモードに遷移
    GPS.sendMTKcommand(225, F(",0"));
    // QZSS（みちびき）をサポートする
    GPS.sendMTKcommand(351, F(",1"));
    // RMCとGGAをともに1サイクルで出力する（1サイクル時間は PMTK220 による）
    // 各項は ",GLL,RMC,VTG,GGA,GSA,GSV,0,0,0,0,0,0,0,0,0,0,0,ZDA,MCHN"
    GPS.sendMTKcommand(314, F(",0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"));}

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
