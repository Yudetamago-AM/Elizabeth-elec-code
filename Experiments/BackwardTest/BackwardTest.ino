#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/*以下lib内自作ライブラリ*/
#include <PinAssign.h>
#include <Motor.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
    /*Motor initialize*/
    motor_init();
    motor_stop();

    /*BNO055 initialize*/
    while (!bno.begin()) {
        //Serial.println(F("BNO055 not ready"));
        delay(100);
    }
    bno.setExtCrystalUse(true);//のっかってるからには使わねば（精度向上）
    //Serial.println(F("BNO055 ready"));
}

void loop() {
    motor_stop();
    Backward();

    delay(5000);
}

void Backward() {
    imu::Vector<3> e_orientation_now;
    getRad(&e_orientation_now);

    while (!((e_orientation_now.z() > -0.3) && (e_orientation_now.z() < 0.3))) {//あとで閾値設定
        //sd_log("or_y:" + String(e_orientation_now.y()));
        //sd_log(F(",,m_f:178178"));
        motor_forward(178, 178);
        getRad(&e_orientation_now);
    }
    motor_stop();
    
    return;
}

float getRad(imu::Vector<3>* e_orientation_now) {
    unsigned long millisTemp = 0;
    //止まっているとCalibrationが悪くなるので，だめだったらちょっと動かす用
    millisTemp = millis();

    uint8_t mag;//磁気センサーのキャリブレーション度を見る．
    imu::Quaternion q_orientation_now;
    
    do {
        bno.getCalibration(NULL, NULL, NULL, &mag);//system, gyro, accel, mag

        //for debug
        /*
        //Serial.print("system: ");
        //Serial.println(system);
        */
        q_orientation_now = bno.getQuat();
        
        if (millisTemp + 1000 < millis()) {
            motor_rotate_R(200);
            delay(300);
            motor_rotate_L(200);
            delay(300);
            motor_stop();
        }
        
    } while (mag < 1);//1, 2，3の時のみループを抜けて出力
    //一時的にすべての場合にしてみる
    
    q_orientation_now.normalize();//重力分を取り除く（vector.h）
    *e_orientation_now = q_orientation_now.toEuler();
}