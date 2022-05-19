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

    /*BNO055 initialize*/
    while (!bno.begin()) {
        Serial.println(F("BNO055 not ready"));
        delay(100);
    }
    bno.setExtCrystalUse(true);//のっかってるからには使わねば（精度向上）
    Serial.println(F("BNO055 ready"));
}

void loop() {
    /*get Quaternion*/
    imu::Quaternion quat = bno.getQuat();
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");

    /*test getRadZ*/
    double radZ = getRadZ();
    Serial.print("getRadZ: ");
    Serial.println(radZ, 10);
    Serial.print(" in deg: ");
    Serial.println(radZ * RADPI, 10);

    /*test accel;*/
    sensors_event_t accelData;
    bno.getEvent(&accelData);
    Serial.print("x: ");
    Serial.print(accelData.acceleration.x(), 6);
    Serial.print(" y: ");
    Serial.print(accelData.acceleration.y(), 6);
    Serial.print(" z: ");
    Serial.println(accelData.acceleration.z(), 6);

    /* Display calibration status for each sensor. */
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);

    /*print temperature*/
    Serial.println(bno.getTemp());
    delay(1000);
}

/*z軸の角度(rad)を取得*/
float getRadZ() {
    imu::Vector<3> e_orientation_now = bno.getQuat().normalize().toEuler();
    return e_orientation_now.z();
}
