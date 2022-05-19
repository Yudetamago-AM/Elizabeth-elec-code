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
//GPS_MTK333X_Serial GPS;

SoftwareSerial testSerial(PIN_GPS_RX, PIN_GPS_TX);

void setup() {
    Serial.begin(9600);
    Serial.println(F("serial begin"));

    // UARTボーレートを 9600bpsに設定する
    //GPS.sendMTKcommand(251, F(",9600"));
    // 1000ms間隔で NMEAを出力する
    //GPS.sendMTKcommand(220, F(",1000"));//1000msごとで十分かは要検証
    //GPS.sendMTKcommand(300, F(",1000,0,0,0,0"));
    // AlwaysLocate モード開始 
    //GPS.sendMTKcommand(225, F(",8"));
    // 上記を解除してスタンダードモードに遷移
    //GPS.sendMTKcommand(225, F(",0"));
    // QZSS（みちびき）をサポートする
    //GPS.sendMTKcommand(351, F(",1"));
    // RMCとGGAをともに1サイクルで出力する（1サイクル時間は PMTK220 による）
    // 各項は ",GLL,RMC,VTG,GGA,GSA,GSV,0,0,0,0,0,0,0,0,0,0,0,ZDA,MCHN"
    //GPS.sendMTKcommand(314, F(",0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"));
    }

void loop() {
    if (testSerial.available()) {
        Serial.println(testSerial.read());
    }
}
