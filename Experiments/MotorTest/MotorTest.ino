#include <Arduino.h>
#include <PinAssign.h>
#include <Motor.h>


void setup() {

}

void loop() {
    motor_foward(100, 100);// pwm 100
    delay(2000);

    motor_stop();
    delay(2000);

    motor_rotate(20);// 20 degree
    delay(2000);

    stop();
}
