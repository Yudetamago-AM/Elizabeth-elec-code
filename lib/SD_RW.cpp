#include "SD_RW.h"

String fileName;
int countFileName;
static bool log_isFirst;
static bool gpslog_isFirst;

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
    while ((SD.exists("L" + String(countFileName) + ".csv")) || (SD.exists("G" + String(countFileName) + ".csv"))) {
        if (countFileName == 1000) {
            Serial.println(F("too much file exists"));
            break;
        }
        countFileName++;
    }
    fileName = String(countFileName) + ".csv";
    //Serial.print(F("filename: "));
    //Serial.println(fileName);

    log_isFirst = true;
    gpslog_isFirst = true;
    Serial.println(F("SD ready"));
} 

void sd_log(String text) {
    File logText = SD.open("L" + fileName, FILE_WRITE);
    //Serial.println("filename: " + fileName);
    if (logText) {
        if (log_isFirst) {
            logText.println(F("millis,log"));
            sd_log(text);
            log_isFirst = false;
        }
        logText.println(String(millis()) + "," + text);
    } else {
        //Serial.println(F("log err"));
    }
    logText.close();
}

void sd_gpslog(String longitude, String latitude, String angle) {
    //あらかじめ1/60000しておいたものを入力
    File logText = SD.open("G" + fileName, FILE_WRITE);
    if (logText) {
        if (gpslog_isFirst) {
            logText.println(F("millis,longitude,latitude,angle"));
            sd_gpslog(longitude, latitude, angle);
            gpslog_isFirst = false;
        }
        logText.println(String(millis()) + "," + longitude + "," + latitude + "," + angle);
        //https://stackoverflow.com/questions/23936246/error-invalid-operands-of-types-const-char-35-and-const-char-2-to-binar
    } else {
        Serial.println(F("glog err"));
    }
    logText.close();
}

/*
認識：自分の地点（gpsLog()），目標地点(log())
制御：方角（），距離（）→GuideGPS→書いた，概ねOK？，GuideDISTにも書いておく．
動作：制御との一致→
*/