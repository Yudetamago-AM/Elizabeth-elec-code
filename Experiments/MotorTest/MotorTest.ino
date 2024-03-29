#include <Arduino.h>
#include <PinAssign.h>
#include <Motor.h>

void setup() {
    /*serial (for debug) initialize*/
    Serial.begin(9600);
    Serial.println(F("Serial begin"));

    motor_init();
    Serial.println("motor init done");   
}

void loop() {
    
    //直進（曲がる）
    motor_forward(100, 100);// pwm 100
    delay(4000);
    motor_stop();
    delay(500);
    
    motor_forward(178, 255);
    delay(3000);
    motor_stop();
    delay(500);
    
    motor_forward(255, 178);
    delay(3000);
    motor_stop();
    delay(500);

    motor_forward(100,0);
    delay(1000);
    motor_stop();

    motor_forward(0, 100);
    delay(1000);
    motor_stop();
    delay(2000);
}