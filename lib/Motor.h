/*モーターを動かすライブラリのヘッダ*/
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PinAssign.h>

void motor_init();
void motor_foward(byte pwmR, byte pwmL);
//void motor_foward_d(int distance); // distance(cm) メモリ食うので廃止
//void motor_back(int pwm); //使わない
void motor_rotate(float angle, float angleNow); //正面を0として±180度
void motor_stop();

#endif