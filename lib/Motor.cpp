#include "Motor.h"

/*
TB67H450FNGのロジック入力は
VIN(H): 2.0V to 5.5V
VIN(L): 0V to 0.8V
なので，pwm = 102以上でないといけない*といったことはなさそうである*．
というのも，VIN(HYS)がmin 100mV, max 300mVとなっているため（であってるのかな…）．
*/

Motor::Motor() {
    pinMode(PIN_MO_R_A, OUTPUT);
    pinMode(PIN_MO_R_B, OUTPUT);

    pinMode(PIN_MO_L_A, OUTPUT);
    pinMode(PIN_MO_L_B, OUTPUT);
}

void Motor::foward(int pwm) {
    digitalWrite(PIN_MO_R_B, LOW);
    digitalWrite(PIN_MO_L_B, LOW);

    analogWrite(PIN_MO_R_A, pwm);
    analogWrite(PIN_MO_L_A, pwm);
}

void Motor::back(int pwm) {
    digitalWrite(PIN_MO_R_A, LOW);
    digitalWrite(PIN_MO_L_A, LOW);

    analogWrite(PIN_MO_R_B, pwm);
    analogWrite(PIN_MO_L_B, pwm);
}

void Motor::stop() {
    // すべてLOW
    digitalWrite(PIN_MO_R_A, LOW);
    digitalWrite(PIN_MO_L_A, LOW);

    digitalWrite(PIN_MO_R_B, LOW);
    digitalWrite(PIN_MO_L_B, LOW);
}

void Motor::foward_d(int distance) {
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


void Motor::rotate(int angle) {
    /*旋回速度，実験で決定する*/
    int rotatePWM = 30;
    /*
    PIN_MO_L/R_Bはそれぞれ5,6ピンを使っているので，デューティー比が若干高くなるそう（下リファレンス）．
    http://www.musashinodenpa.com/arduino/ref/index.php?f=0&pos=2153

    具体的には，5/6ピンから977Hz
    9/10ピンから490Hz
    3/11ピンから490Hzという3つがあり，490*2 = 980のためと思われる．
    （一応）その調整をする．
    */
    int offsetPWM = 1;//（およそ）1秒(50ms*20)に1回**offsetPWMの値**を引いたPWMでうごかす
    int count = 0;

    //いい感じに9軸センサー呼び出す感じ
    //cf1. https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
    //cf2. https://forums.adafruit.com/viewtopic.php?f=19&t=91723&p=462337#p462337
    /*
    ↑では，
    sensors_event_t rotate;
    bno.getEvent(&rotate);
    みたいに書いてあるが，
    https://learn.adafruit.com/using-the-adafruit-unified-sensor-driver/how-does-it-work#void-getevent-sensors-event-t-star
    にあるように，Quternionはgeteventでは取得出来ない（=Raw Sensor Dataとして取得する必要がある）
    （なんのための"Unified" Sensor Systemだ？？，下手に使えんでないか）
    */
    //cf3. http://l52secondary.blog.fc2.com/blog-entry-50.html
    
    /*
    そのままEulerで取得するとpitch(y)かroll(x)軸が±45度を超えると，まともに使えるデータでなくなるから
    クォータニオンで値を取得してオイラー角に変換する
    ちなみに，（正）「クォータニオン」（誤）「クォータ二オン」，（誤）はニが2になってる，おのれATOK！
    */
    imu::Quaternion q_orientation_now = bno.getQuat();
    q_orientation_now.normalize;//重力分を取り除く（vector.h）

    /*入れ替え*/
    float temp = q_orientation_now.x();
    q_orientation_now.x() = -q_orientation_now.y();
    q_orientation_now.y() = temp;
    orientation.z() = -q_orientation_now.z();

    imu::Vector<3> e_orientation_now = q_orientation_now.toEuler();
    if (angle > 0) {
        /*右は後転，左は正転*/
        while ((-180/M_PI * q_orientation_now.z()) <= angle) {
            digitalWrite(PIN_MO_L_B, LOW);
            digitalWrite(PIN_MO_R_A, LOW);

            analogWrite(PIN_MO_L_A, rotatePWM);
            if (count < 20) {
                analogWrite(PIN_MO_R_B, rotatePWM);
                count++;
            } else {
                analogWrite(PIN_MO_R_B, rotatePWM - offcetPWM);
                count = 0;
            }
        }
    } else if (angle < 0) {
        while ((-180/M_PI * q_orientation_now.z()) <= angle) {
            /*右は正転，左は後転*/
            digitalWrite(PIN_MO_L_A, LOW);
            digitalWrite(PIN_MO_R_B, LOW);

            analogWrite(PIN_MO_L_B, rotatePWM);
            if (count < 20) {
                analogWrite(PIN_MO_R_A, rotatePWM);
                count++;
            } else {
                analogWrite(PIN_MO_R_A, rotatePWM - offsetPWM);
                count = 0;
            }
        }
    } else {
        return;
    }
}