#include "Motor.h"

/*
TB67H450FNGのロジック入力は
VIN(H): 2.0V to 5.5V
VIN(L): 0V to 0.8V
なので，pwm = 102以上でないといけない*といったことはなさそうである*．
というのも，VIN(HYS)がmin 100mV, max 300mVとなっているため（であってるのかな…）．
*/

void motor_init() {
    pinMode(PIN_MO_R_A, OUTPUT);
    pinMode(PIN_MO_R_B, OUTPUT);

    pinMode(PIN_MO_L_A, OUTPUT);
    pinMode(PIN_MO_L_B, OUTPUT);
}

/*あとで全部これに統一*/
void motor_forward(byte pwmR, byte pwmL) {
    analogWrite(PIN_MO_R_A, pwmR);
    analogWrite(PIN_MO_L_A, pwmL);

    digitalWrite(PIN_MO_R_B, LOW);
    digitalWrite(PIN_MO_L_B, LOW);
}

void motor_backward(byte pwmR, byte pwmL) {
    digitalWrite(PIN_MO_R_A, LOW);
    digitalWrite(PIN_MO_L_A, LOW);

    analogWrite(PIN_MO_R_B, pwmR);
    analogWrite(PIN_MO_L_B, pwmL);
}

void motor_stop() {
    // すべてLOW
    digitalWrite(PIN_MO_R_A, LOW);
    digitalWrite(PIN_MO_L_A, LOW);

    digitalWrite(PIN_MO_R_B, LOW);
    digitalWrite(PIN_MO_L_B, LOW);
}

/*
void motor_forward_d(int distance) {
    int time, operatingTime;
    const int tireOneRotate = 40.84;// cm （タイヤ1回転は13*pi = 40.84cmくらい？）
    const int timeOfOneRotate = 5;//実験で良い感じの値を決める．A定電流制御でトルク一定のはずだから，定数としていいはず．
    operatingTime = (distance / tireOneRotate) * timeOfOneRotate;

    forward(110);

    if((time + operatingTime) <= millis()) {
        motor_stop();
    }
    return;
    //ほんとうなら，つど加速度の値と時間を記録しておいて，進んだ距離を推定して，
    //フィードバックすべきだが，めんどくさいし，計算量も多くなるので，余裕があったらということにしておく
}
*/

//motor_rotateはmainに．→回すだけのを作ることにした（スタック処理とかでも使えるから）．

/*右旋回*/
void motor_rotate_R(byte pwm) {
    //右は正転
    digitalWrite(PIN_MO_R_A, LOW);
    analogWrite(PIN_MO_R_B, pwm);

    //左は後転
    digitalWrite(PIN_MO_L_B, LOW);
    analogWrite(PIN_MO_L_A, pwm);
}

void motor_rotate_L(byte pwm) {
    //右は後転
    digitalWrite(PIN_MO_R_B, LOW);
    analogWrite(PIN_MO_R_A, pwm);

    //左は正転
    digitalWrite(PIN_MO_L_A, LOW);
    analogWrite(PIN_MO_L_B, pwm);
}