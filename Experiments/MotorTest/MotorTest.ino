#include <Arduino.h>
#include "./PinAssign.h"
#include "./Motor.h"

void setup() {
    Motor();

}

void loop() {
    foward(100);
    delay(2000);

    stop();
    delay(2000);

    back(50);
    delay(2000);
    
    stop();
}
