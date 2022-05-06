#include "SD_RW.h"

SD_RW::SD() {
    pinMode(PIN_SD_CS);
}

SD_RW::init() {
    //起動する度，新しい名前のファイル作る（連番）
    //ファイル名は8.3まで（リファレンスより）
    //それをString fileNameに保存する
}

SD_RW::write(String text) {
    SD.open(/log.txt);
    //後で書く
}