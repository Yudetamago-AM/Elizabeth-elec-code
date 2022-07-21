#include <Arduino.h>
#include <PinAssign.h>
#include <Motor.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD_RW.h>

byte phase = 3;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
unsigned long millisTemp = 0;
unsigned long millisTemp2 = 0;

void setup() {
    /*Motor initialize*/
    motor_init();
    motor_stop();

    /*serial (for debug) initialize*/
    Serial.begin(9600);
    Serial.println(F("Serial begin"));

    /*Distance sensor initialize*/
    pinMode(PIN_DIST_TRIG, OUTPUT);
    pinMode(PIN_DIST_ECHO, INPUT);
    digitalWrite(PIN_DIST_TRIG, LOW);

    /*BNO055 initialize*/
    while (!bno.begin()) {
        Serial.println(F("BNO055 not ready"));
        delay(100);
    }
    bno.setExtCrystalUse(true);//のっかってるからには使わねば（精度向上）
    Serial.println(F("BNO055 ready"));

    /*SD initialize*/
    sd_init();
}
void loop() {
switch (phase)
    {
    case 3:
        Backward();
        GuideDIST();
        //stack();
        break;
    case 4:
        Goal();
        //stack();
        break;
    case 5://ゴール後
        //GPS.check();
        delay(10000);
        break;
    }
}

void GuideDIST() {
    //Serial.println(F("gidD bgn"));
    /*精密誘導が上手くいかない場合
    if (goalCount > 4) {
        phase = 5;
        return;
    }
    */

    sd_log(F("DIST Guide"));
    imu::Vector<3> e_orientation_now;
    getRad(&e_orientation_now);

    byte dist[18], i;//あるいは，印付けてみるとか？
    dist[0] = 200;
    rotate(e_orientation_now.x() - 1.57);//90度左回転
    
    for (i = 0; i < 5; i++) {
        getRad(&e_orientation_now);
        dist[i] = Distance() / 2;//2cm単位 
        rotate(e_orientation_now.x() + 0.175);
        delay(10);
        /*
        for ((dist[i] < 200) && (abs(dist[i] - dist[i - 1]) > 100)) {
            sd_log("dist:" + String(dist[i]))
        }
        */
    }

    for (i = 0; i < 5; i++) {
        if((dist[i] < 200) && (((dist[i] - dist[i - 1]) < 10) || ((dist[i] - dist[i - 1]) > -10))) {
            //ログ
            sd_log("dist_num:" + String(i));//distance direction
            sd_log("dist:" + String(dist[i]));

            //制御
            getRad(&e_orientation_now);
            rotate(e_orientation_now.x() - (0.175 * (18 - i)));
            delay(10);
            motor_forward(255, 255);
            //millisTemp2 = millis();
            sd_log(",,m_f:255255");
            delay(850);
            /*
            while (1) {
                if (millisTemp2 + 1500 <= millis()) {
                    motor_stop;
                    break;
                }
            }
            */
            break;
        }
    }

    //Serial.println(F("done"));
    phase = 4;//is goal?
}

void Goal() {
    //Serial.println(F("isGoal begin"));
    sd_log("isGoal");

    bool gpsComplete = false;
    bool right = false, left = false;
    double direction, distance;

    /*
    getGPS(&direction, &distance);
    if (distance < 400) {//gpsで5m以内
        gpsComplete = true;
    } else if (goalCount > 4) {
        phase = 2;
        return;
    }
    */
    gpsComplete = true;

    //0m判定
    if (gpsComplete && (Distance() < 12)) {
        motor_rotate_R(100);
        delay(200);
        motor_stop();
        if (Distance() < 12) right = true;

        motor_rotate_L(100);
        delay(200);
        motor_stop();
        if (Distance() < 12) left = true;
    } else {
        phase = 4;
    }

    if (gpsComplete && right && left) {
        phase = 5;//guide done -- wait for power off
        //Serial.println(F("0m goal!"));
    }
    //goalCount++;
    return;
}

void Backward() {
    imu::Vector<3> e_orientation_now;
    getRad(&e_orientation_now);

    while (!((e_orientation_now.z() > -0.5) && (e_orientation_now.z() < 1))) {//あとで閾値設定
        sd_log("or_y:" + String(e_orientation_now.y()));
        sd_log(",,m_f:178178");
        motor_forward(178, 178);
        getRad(&e_orientation_now);
    }
    motor_stop();
    
    return;
}

double Distance() {
    /*
    距離(cm)を返す
    一応，九軸センサー内蔵の温度センサーで温度を見て，校正しているが，だめっぽかったら15℃として計算してる
    これは, ambient temperature なのか， sensor temperatureなのか，よくわからん（データーシートにも混在）→実験してみる
    ！！メモリ節約のため，25度で計算
    */
    int Duration;
    double Distance;

    digitalWrite(PIN_DIST_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_DIST_TRIG, LOW);

    Duration = pulseIn(PIN_DIST_ECHO, HIGH, 23200);//400cm以上の場合0を返す
    if (Duration > 0) {
        //Distance = Duration * (0.6 * temp + 331.5) / 20000;
        // D(cm) = T(μS) × 1/2(片道) × 340(m/s) × 100(cm/m) × 1/1000000(μS/S)
        // https://nobita-rx7.hatenablog.com/entry/27884818
        // D = T * 0.00005 * c(m/s) = T / 20000
        // c(近似) = 0.6 * temp + 331.5　

        //Distance = Duration / 58.8;// 15℃ 1013hpa
        // D = T * 0.017 = T / 58.8

        Distance = Duration / 57.6;
        // 25度の音速346.75(m/s)なので347として計算
        // D = T * 0.01735 = T / 57.6
    } else {
        Distance = 400;
    }
    return Distance;
}

/*Rotate to the ABSOLUTE angle (in euler (rad))*/
void rotate(double angle) {
    unsigned long millisTemp2 = 0;

    millisTemp2 = millis();
    /*
    PIN_MO_L/R_Bはそれぞれ5,6ピンを使っているので，デューティー比が若干高くなるそう（下リファレンス）．
    http://www.musashinodenpa.com/arduino/ref/index.php?f=0&pos=2153

    具体的には，5/6ピンから977Hz
    9/10ピンから490Hz
    3/11ピンから490Hzという3つがあり，490*2 = 980のためと思われる．
    （一応）その調整をする．
    */
    /*
    int offsetPWM = 1;//（およそ）1秒(50ms*20)に1回**offsetPWMの値**を引いたPWMでうごかす
    int count = 0;
    // メモリないのでやめにした
    */
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

    /*
    imu::Quaternion q_orientation_now = bno.getQuat();
    q_orientation_now.normalize();//重力分を取り除く（vector.h）
    */
    imu::Vector<3> e_orientation_now;
    getRad(&e_orientation_now);
    /*入れ替え
    // x to -y
    // y to x
    // z to -z
    double temp = q_orientation_now.x();
    q_orientation_now.x() = -q_orientation_now.y();
    q_orientation_now.y() = temp;
    q_orientation_now.z() = -q_orientation_now.z();
    //おそらく不要
    */
    /*
    imu::Vector<3> e_orientation_now = q_orientation_now.toEuler();
    double angleNow = radPI * e_orientation_now.z();
    */
    float destAngle;//destまでのangle
    if (angle > 0) destAngle = angle + e_orientation_now.x();
    else destAngle = angle - e_orientation_now.x();
    
    while (destAngle > 3.14) {
        destAngle -= 3.141592;
    }
    while (destAngle < -3.14) {
        destAngle += 3.141592;
    }
    //Serial.print(F("destAngle: "));
    //Serial.println(destAngle);
    /*旋回*/
    /*
    while (e_orientation_now.x() <= destAngle) {
        if (destAngle > 0) {
            motor_rotate_L(110);
            if ((e_orientation_now.x() <= destAngle) || dest)
        }
        else if (destAngle < 0) motor_rotate_R(110);

        if (millisTemp2 + 5000 <= millis()) {
            motor_stop();
            return;
        }

        delay(12);//9軸は100Hz=10msごと更新だが余裕を見る
        getRad(&e_orientation_now);
    }
    */
    
    if (destAngle > 0) {
        while (e_orientation_now.x() <= destAngle) {
            motor_rotate_L(255);
            sd_log(",,m_r_L:255");
            
            if (millisTemp2 + 5000 <= millis()) break;
            delay(1);
            getRad(&e_orientation_now);
        }
        motor_rotate_R(255);
        delay(10);
        motor_stop();
        return;
    }
    if (destAngle < 0) {
        while (e_orientation_now.x() >= destAngle) {
            motor_rotate_R(255);
            sd_log(",,m_r_R:255");
            
            if (millisTemp2 + 5000 <= millis()) break;
            delay(1);
            getRad(&e_orientation_now);
        }
        motor_rotate_L(255);
        delay(10);
        motor_stop();
        return;
    }
    
    return;
}


float getRad(imu::Vector<3>* e_orientation_now) {
    unsigned long millisTemp = 0;
    //止まっているとCalibrationが悪くなるので，だめだったらちょっと動かす用
    millisTemp = millis();

    uint8_t mag;//磁気センサーのキャリブレーション度を見る．
    imu::Quaternion q_orientation_now;
    
    do {
        bno.getCalibration(NULL, NULL, NULL, &mag);//system, gyro, accel, mag

        //for debug
        /*
        //Serial.print("system: ");
        //Serial.println(system);
        */
        q_orientation_now = bno.getQuat();
        
        
        if (millisTemp + 1000 < millis()) {
            motor_rotate_R(200);
            delay(300);
            motor_rotate_L(200);
            delay(300);
            motor_stop();
        }
        
    } while (mag < 1);//1, 2，3の時のみループを抜けて出力
    //一時的にすべての場合にしてみる
    
    q_orientation_now.normalize();//重力分を取り除く（vector.h）
    *e_orientation_now = q_orientation_now.toEuler();
}