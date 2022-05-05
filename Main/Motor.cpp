#include "./Motor.h"

/*
TB67H450FNGのロジック入力は
VIN(H): 2.0V to 5.5V
VIN(L): 0V to 0.8V
なので，pwm = 102以上でないといけない*といったことはなさそうである*．
というのも，VIN(HYS)がmin 100mV, max 300mVとなっているため（であってるのかな…）．
*/

void Motor() {
    pinMode(PIN_MO_R_A, OUTPUT);
    pinMode(PIN_MO_R_B, OUTPUT);

    pinMode(PIN_MO_L_A, OUTPUT);
    pinMode(PIN_MO_L_B, OUTPUT);
}

void foward(int pwm) {
    digitalWrite(PIN_MO_R_B, LOW);
    digitalWrite(PIN_MO_L_B, LOW);

    analogWrite(PIN_MO_R_A, pwm);
    analogWrite(PIN_MO_L_A, pwm);
}

void back(int pwm) {
    digitalWrite(PIN_MO_R_A, LOW);
    digitalWrite(PIN_MO_L_A, LOW);

    analogWrite(PIN_MO_R_B, pwm);
    analogWrite(PIN_MO_L_B, pwm);
}

void stop() {
    // すべてLOW
    digitalWrite(PIN_MO_R_A, LOW);
    digitalWrite(PIN_MO_L_A, LOW);

    digitalWrite(PIN_MO_R_B, LOW);
    digitalWrite(PIN_MO_L_B, LOW);
}

void foward_d(int distance) {
    int time, operatingTime;
    const int tireOneRotate = 40.84;// cm （タイヤ1回転は13*pi = 40.84cmくらい？）
    const int timeOfOneRotate = 5;//実験で良い感じの値を決める．A定電流制御でトルク一定のはずだから，定数としていいはず．
    operatingTime = (distance / tireOneRotate) * timeOfOneRotate;

    foward(110);

    if((time + operatingTime) <= millis()) {
        stop();
    }
    /*
    ほんとうなら，つど加速度の値と時間を記録しておいて，進んだ距離を推定して，
    フィードバックすべきだが，めんどくさいし，計算量も多くなるので，余裕があったらということにしておく
    */
}


void rotate(int angle) {
    //いい感じに9軸センサー呼び出す感じ
    if (angle > 0) {

    } else if (angle < 0) {

    } else {
        return 0;
    }
}
