/*モーターを動かすライブラリのヘッダ*/
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "PinAssign.h"

class Motor {
public:
    Motor();

    foward(int pwm);
    foward_d(int distance); // distance(cm)
    back(int pwm);
    rotate(int angle); //正面を0として±180度
    stop();
}

#endif