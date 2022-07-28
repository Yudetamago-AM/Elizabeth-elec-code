/*モーターを動かすライブラリのヘッダ*/
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <PinAssign.h>

void motor_init();
void motor_forward(byte pwmR, byte pwmL);
//void motor_forward_d(int distance); // distance(cm) メモリ食うので廃止
void motor_back(byte pwmR, byte pwmL); //使わない
//void motor_rotate(float angle); //正面を0として±pi rad
//このライブラリはただモーターを回すだけ，ということにするから，rotateはMain.inoに移行→ただ回すだけのmotor_rotateをつくる
void motor_rotate_R(byte pwm);
void motor_rotate_L(byte pwm);
void motor_stop();

#endif