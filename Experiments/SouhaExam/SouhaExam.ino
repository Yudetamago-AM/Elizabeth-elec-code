#include <Arduino.h>
#include <PinAssign.h>
#include <Motor.h>

void setup() {
    motor_init();
}

void loop() {
    //直進（曲がる）
    motor_forward(255, 255);// pwm 100
    delay(20000);
    motor_stop();
    delay(50);
}