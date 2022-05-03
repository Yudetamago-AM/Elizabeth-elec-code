#include <Arduino.h>
#include "PinAssign.h"
#include "Motor.hpp"

Motor motor = Motor();

void setup() {

}

void loop() {
    motor.foward(100);
    delay(2000);
    motor.stop();
    motor.back(50);
    delay(2000);
    motor.stop();
}
