#include <Arduino.h>
#include <PinAssign.h>
#include <Motor.h>

Motor motor = Motor();

void setup() {

}

void loop() {
    motor.foward(100);// pwm 100
    delay(2000);

    motor.stop();
    delay(2000);

    motor.back(50);// pwm 50
    delay(2000);

    motor.rotate(20);// 20 degree
    delay(2000);

    stop();
}
