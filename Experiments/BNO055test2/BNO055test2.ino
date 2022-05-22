#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/*以下lib内自作ライブラリ*/
#include <PinAssign.h>
#include <Motor.h>

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
    /*get Quaternion
    imu::Quaternion quat = bno.getQuat();
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.println("\t\t");
    */

    /*test getRad*/
    /*
    imu::Quaternion q_orientation_now = bno.getQuat();
    q_orientation_now.normalize();//重力分を取り除く（vector.h）
    imu::Vector<3> e_orientation_now = q_orientation_now.toEuler();
    */
    imu::Vector<3> e_orientation_now;
    //imu::Quaternion e_orientation_now;
    getRad(&e_orientation_now);

    /*
    Serial.println("orientation: ");
    Serial.print(" x: ");
    Serial.println(e_orientation_now.x() * RADPI, 10);
    Serial.print(" y: ");
    Serial.println(e_orientation_now.y() * RADPI, 10);
    Serial.print(" z: ");
    Serial.println(e_orientation_now.z() * RADPI, 10);
    Serial.println("");
    */
    Serial.println("orientation: ");
    Serial.print(" x: ");
    Serial.println(e_orientation_now.x(), 10);
    Serial.print(" y: ");
    Serial.println(e_orientation_now.y(), 10);
    Serial.print(" z: ");
    Serial.println(e_orientation_now.z(), 10);
    Serial.println("");

    /*test accel;
    sensors_event_t accelData;
    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    //VECTOR_ACCELEROMETERだと，重力加速度も入ってる．
    Serial.println("Accel:");
    Serial.print(" x: ");
    Serial.println(accelData.acceleration.x, 6);
    Serial.print(" y: ");
    Serial.println(accelData.acceleration.y, 6);
    Serial.print(" z: ");
    Serial.println(accelData.acceleration.z, 6);
    /*
    imu::Vector<3> accelData = bno.getVector(Adafruit_BNO055::VECTOR_LINERACCEL);
    Serial.println("accel:");
    Serial.print("x: ");
    Serial.print(accelData.x(), 6);
    Serial.print(" y: ");
    Serial.print(accelData.y(), 6);
    Serial.print(" z: ");
    Serial.println(accelData.z(), 6);
    */ 
    
    /*print temperature
    //センサーの温度だった
    Serial.print("temp: ");
    Serial.println(bno.getTemp());
    */
    delay(200);
}

float getRad(imu::Vector<3>* e_orientation_now) {
    //止まっているとCalibrationが悪くなるので，だめだったらちょっと動かす用
    //millisTemp = millis();

    uint8_t system;
    imu::Quaternion q_orientation_now;
    
    do {
        bno.getCalibration(&system, NULL, NULL, NULL);

        //for debug
        Serial.print("system: ");
        Serial.println(system);
        q_orientation_now = bno.getQuat();
        /*
        if (millisTemp + 1000 < millis()) {
            motor_foward(50, 50);
            delay(100);
            motor_stop();
        }
        */
    } while (system < 1);//1, 2，3の時のみループを抜けて出力
    
    q_orientation_now.normalize();//重力分を取り除く（vector.h）
    *e_orientation_now = q_orientation_now.toEuler();
}
/*
//Quaternion
//Z軸が回転
float getRadr(imu::Quaternion* e_orientation_now) {
    imu::Quaternion q_orientation_now = bno.getQuat();
    q_orientation_now.normalize();//重力分を取り除く（vector.h）
    *e_orientation_now = q_orientation_now;
}
*/
