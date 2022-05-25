#include "SD_RW.h"

String fileName;
int countFileName;
bool log_isFirst = true;
bool gpslog_isFirst = true;

void sd_init() {
    pinMode(PIN_SD_CS, OUTPUT);

    while (!SD.begin(PIN_SD_CS)){
        Serial.println(F("SD not ready"));
        delay(100);
    }

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
    fileName = String(countFileName) + ".csv";
    //Serial.print(F("filename: "));
    //Serial.println(fileName);
    Serial.println(F("SD ready"));
}

void sd_log(String text) {
    File logText = SD.open("l_" + fileName, FILE_WRITE);
    //Serial.println("filename: " + fileName);
    if (logText) {
        if (log_isFirst == true) {
            logText.println(F("millis,log"));
            sd_log(text);
            log_isFirst = false;
        } else {
            logText.println(String(millis()) + "," + text);
        }
    } else {
        Serial.println(F("SD_RW log error"));
    }
    logText.close();
}

//メモリ節約のため，ポインタ使って書きなおす
void sd_gpslog(bcdtime_t* bcdtime, int32_t* longitude, int32_t* latitude, float angle) {//！angleだけポインタじゃない
    //あらかじめ1/60000.0しておいたものを入力
    File logText = SD.open("gps-" + fileName, FILE_WRITE);
    if (logText) {
        if (gpslog_isFirst == true) {
            logText.println(F("millis,bcdtime,longitude,latitude,angle"));
        } else {
            logText.println(String(millis()) + "," + String(*bcdtime) + "," + String(*longitude) + "," + String(*latitude) + "," + String(angle));
        }
        gpslog_isFirst = false;
    } else {
        Serial.println(F("SD_RW gpslog error"));
    }
    logText.close();
}

/*
認識：自分の地点（gpsLog()），目標地点(log())
制御：方角（），距離（）→GuideGPS，GuideDISTを書くときに，ついでに書く
動作：制御との一致→
*/