#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/*以下lib内自作ライブラリ*/
#include <PinAssign.h>
#include <SD_RW.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
    /*serial (for debug) initialize*/
    Serial.begin(9600);
    Serial.println(F("Serial begin"));

    /*sd init*/
    sd_init();

    /*BNO055 initialize*/
    while (!bno.begin()) {
        Serial.println(F("BNO055 not ready"));
        delay(100);
    }
    bno.setExtCrystalUse(true);//のっかってるからには使わねば（精度向上）
    Serial.println(F("BNO055 ready"));
    
    byte i;
    for (i = 0; i < 5; i++) {
        //書き込み
        sd_log("hello sd card!>" + String(i));//ここはなぜかFオプションが効かない
        delay(10);
    }

    Serial.println(F("log done!"));

}

void loop(){

        /*test accel;*/
    sensors_event_t accelData;
    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    //VECTOR_ACCELEROMETERだと，重力加速度も入ってる．
    Serial.println("Accel:");
    Serial.print(" x: ");
    Serial.println(accelData.acceleration.x, 6);
    sd_log("accel x:");
    sd_log(String(accelData.acceleration.x, 6));
    Serial.print(" y: ");
    Serial.println(accelData.acceleration.y, 6);
    sd_log("accel y:");
    sd_log(String(accelData.acceleration.y, 6));
    Serial.print(" z: ");
    Serial.println(accelData.acceleration.z, 6);
    sd_log("accel z:");
    sd_log(String(accelData.acceleration.z, 6));

     /* Display calibration status for each sensor. */
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);
    Serial.println("");
    delay(900);//だいたい1秒ごと
}

