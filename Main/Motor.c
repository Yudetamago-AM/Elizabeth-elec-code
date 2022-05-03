#include "Motor.h"

/*
TB67H450FNGのロジック入力は
VIN(H): 2.0V to 5.5V
VIN(L): 0V to 0.8V
なので，pwm = 102以上でないといけない*といったことはなさそうである*．
というのも，VIN(HYS)がmin 100mV, max 300mVとなっているため（であってるのかな…）．
*/

Motor::Motor() {
    pinMode(PIN_MO_R_LOW, OUTPUT);
    pinMOde(PIN_MO_L_LOW, OUTPUT);
}

void Motor::foward(int pwm) {
    digitalWrite(PIN_MO_R_LOW, LOW);
    digitalWrite(PIN_MO_R_LOW, LOW);

    analogWrite(PIN_MO_R_PWM, pwm);
    analogWrite(PIN_MO_L_PWM, pwm);
}

void Motor::foward_d(int distance) {
    int time = millis(), operatingTime;
    operatingTime 
    foward(110);
    if(time + )
}