#ifndef PIN_ASSIGN
#define PIN_ASSIGN
/*
Arduinoブートローダであることを意識．
いちおう命名規則として，すべて大文字とする．

example:
digitalWrite(PIN_NICHROME, HIGH);
とすれば，digital pin 7がHIGHになります．
*/

/*
GPS
標準のI2Cバス（TX:0, RX:1）
*/

/*9軸センサー*/
/*
Adafruitのライブラリですでにdefineされてるはず
SDA A4
SCL A5
*/

/*距離センサー*/
#define PIN_DIST_ECHO 3
#define PIN_DIST_TRIG 4

/*SDカード*/
#define PIN_SD_CS 8
#define PIN_SD_CMD 11 //MOSI
#define PIN_SD_DATA 12 //MISO
#define PIN_SD_CLK 13 //SCL

/*右モーター(M1)*/
#define PIN_MO_R_A 9 // 回路図でPWM
#define PIN_MO_R_B 5

/*左モーター(M2)*/
#define PIN_MO_L_A 10 // 回路図でPWM
#define PIN_MO_L_B 6

/*フライトピン*/
#define PIN_FLIGHT 4

/*ニクロム線*/
#define PIN_NICHROME 7

/*定数*/
#define RADPI 57.2957795131

#endif