/*モーターを動かすライブラリのヘッダ*/
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "./PinAssign.h"

        int Motor();
        
        void foward(int pwm);
        void foward_d(int distance);//distance(cm)
        void back(int pwm);
        void rotate(int angle);//正面を0として±180度
        void stop();

#endif