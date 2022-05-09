#include "SD_RW.h"

SD_RW::SD_RW() {
    pinMode(PIN_SD_CS);
}

SD_RW::init() {
    //起動する度，新しい名前のファイル作る（連番）
    //ファイル名は8.3まで（リファレンスより）
    //それをString fileNameに保存する
    countFileName = 0;
    while (SD.exists(String(countFileName) + ".txt")) {
        if (countFileName == 1000) {
            Serial.println(F("too much file exists"));
            break;
        }
        countFileName++;
    }
    fileName = String(countFileName);
    fileName.concat(".txt");
    Serial.println(F("SD ready"));
}

SD_RW::log(String text) {
    static int count = 0;
    File logText = SD.open("log_" + fileName, FILE_WRITE);
    if (logText) {
        if (count == 0) {
            logText.println("millis,log");
        } else {
            logText.println(String(millis()) + "," + text);
        }
    } else {
        Serial.println(F("SD_RW log error"));
    }
}

SD_RW::gpsLog(bcdtime_t bcdtime, int32_t longitude, int32_t latitude, int32_t speed, float angle) {//あらかじめ1/60000.0しておく
    static int count = 0;
    File logText = SD.open("gps_" + fileName, FILE_WRITE);
    if (logText) {
        if (count == 0) {
            logText.println("millis,bcdtime,longitude,latitude,angle");
        } else {
            logText.println("millis," + String(bcdtime) + "," + String(longitude) + "," + String(latitude) + "," + String(angle));
        }
    } else {
        Serial.println(F("SD_RW gpslog error"));
    }
}