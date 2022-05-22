/*
あらかじめ，
https://github.com/askn37/Futilities
https://github.com/askn37/GPS_MTK333X
https://github.com/adafruit/Adafruit_Sensor
https://github.com/adafruit/Adafruit_BusIO
https://github.com/adafruit/Adafruit_BNO055
をライブラリに読み込んでおくこと．
*/

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <GPS_MTK333X_SoftwareSerial.h>
//#include <GPS_MTK333X_Serial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/*以下lib内自作ライブラリ*/
#include <PinAssign.h>
#include <Motor.h>
#include <SD_RW.h>

byte phase = 0;
/*重要！！電源オンからのタイマー，ミリ秒単位で指定*/
const unsigned long Timer = 600000; //600 * 1000, 600秒 は 10分
/*フライトピン解放からのタイマー*/
const unsigned int flightPinTimer = 50000;//50s
unsigned long flightPinMillis;
/*重要！！ゴールのGPS座標*/
const float goal_longitude = 140.026945; //経度
const float goal_latitude = 40.211944; //緯度

unsigned long millisTemp = 0;

GPS_MTK333X_SoftwareSerial GPS(PIN_GPS_RX, PIN_GPS_TX);
//GPS_MTK333X_Serial GPS;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
    /*ニクロム線の初期設定*/
    pinMode(PIN_NICHROME, OUTPUT);
    digitalWrite(PIN_NICHROME, LOW);

    /*serial (for debug) initialize*/
    Serial.begin(9600);
    Serial.println(F("Serial begin"));

    /*FOR DEBUG!! mem check*/
    /*不要なときはコメントアウト！！*/
    // 5/16時点で723bytesあまってる感じ
    /*
    Serial.print(F("Free memory="));
    Serial.print(freeRam(), DEC);
    Serial.println(F("[bytes]"));
    */

    /*SD initialize*/
    sd_init();

    /*BNO055 initialize*/
    while (!bno.begin()) {
        Serial.println(F("BNO055 not ready"));
        delay(100);
    }
    bno.setExtCrystalUse(true);//のっかってるからには使わねば（精度向上）
    Serial.println(F("BNO055 ready"));

    /*GPS initialize*/
    while (!GPS.begin(9600)) {
        Serial.println(F("GPS not ready"));
        delay(100);
    }
    // UARTボーレートを 9600bpsに設定する
    GPS.sendMTKcommand(251, F(",9600"));
    // 500ms間隔で NMEAを出力する
    GPS.sendMTKcommand(220, F(",500"));//500msごとで十分かは要検証
    // AlwaysLocate モード開始 
    //GPS.sendMTKcommand(225, F(",8"));
    // 上記を解除してスタンダードモードに遷移
    //GPS.sendMTKcommand(225, F(",0"));
    // QZSS（みちびき）をサポートする
    GPS.sendMTKcommand(351, F(",1"));
    // RMCとGGAをともに1サイクルで出力する（1サイクル時間は PMTK220 による）
    // 各項は ",GLL,RMC,VTG,GGA,GSA,GSV,0,0,0,0,0,0,0,0,0,0,0,ZDA,MCHN"
    GPS.sendMTKcommand(314, F(",0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"));
    Serial.println(F("GPS ready"));

    /*Distance sensor initialize*/
    pinMode(PIN_DIST_TRIG, OUTPUT);
    pinMode(PIN_DIST_ECHO, INPUT);
    digitalWrite(PIN_DIST_TRIG, LOW);
    
    /*Motor initialize*/
    motor_init();

    /*flight pin initialize*/
    pinMode(PIN_FLIGHT, INPUT);
}

void loop() {
    //フライトピン
    if (PIN_FLIGHT == LOW) flightPinMillis = millis();

    //GPS受信バッファあふれ防止
    GPS.check();
    switch (phase)
    {
    case 0:
        Landing();
        break;
    case 1:
        GuideGPS();
        break;
    case 2:
        GuideDIST();
        break;
    case 3:
        Goal();
        break;
    case 4://ゴール後
        GPS.check();
        delay(10000);
        break;
    }
}

/*各シークエンス*/
void Landing() {
    if (isLanded()) {
        // 前にうごかしたりパラシュート切り離したり…
        /*ニクロム線のカット*/
        digitalWrite(PIN_NICHROME, HIGH);
    }
    phase = 1;
}

/*guide to the goal using GPS*/
void GuideGPS() {
    //方位・距離を計算
    double direction, distance, angle;//方位，距離，角度の差
    
    getGPS(&direction, &distance);
    
    //ゴールまで4m以内なら，精密誘導へ
    if (distance <= 4) {
        motor_stop();
        phase = 2;
        return;
    }
    
    //方位をみて制御
    imu::Vector<3> e_orientation_now;
    getRad(&e_orientation_now);
    angle = e_orientation_now.x() - direction;

    bool isPlus = false; //trueなら正，falseなら負
    if (angle > 0) isPlus = true;

    angle = abs(angle);
    if (angle <= 0.26) motor_foward(255, 255); //だいたい15度
    else if (angle <= 1.3 && isPlus) motor_foward(178, 255);//だいたい75度，ゴールが右手，出70%くらい
    else if (angle <= 1.3) motor_foward(255, 178);
    else if (angle <= 3.14 && isPlus) motor_foward(76, 255);
    else if (angle <= 3.15) motor_foward(255, 75);//わざと3.15，というのも，PI = 3.141592...で，3.14ならカバーできない角度が生まれるため
    
    delay(2000);
}

void GuideDIST() {
    byte dist[18], i;//あるいは，印付けてみるとか？

    rotate(-1.57);//90度左回転
    for (i = 0; i < 18; i++) {
        dist[i] = Distance() / 2;//2cm単位 
        rotate(0.175);
        delay(10);//これは吉と出るか凶と出るか…要実験！！
    }
    for (i = 0; i < 18; i++) {
        if((dist[i] < 200) && (abs(dist[i] - dist[i - 1]) < 10)) {
            rotate(0.175 * (18 - i));
            motor_foward(178, 178);
            delay(2000);
            motor_stop();
            break;
        }
    }
    Goal();
}

void Goal() {
    //4m以内判定
    bool gpsComplete = false;
    double direction, distance;

    getGPS(&direction, &distance);
    if (distance < 4) {
        gpsComplete = true;
        GuideDIST();
    }

    //0m判定
    bool right = false, left = false;
    if (gpsComplete && (Distance() < 5)) {
        rotate(0.1);
        if (Distance() < 5) right = true;
        rotate(-0.2);
        if (Distance() < 5) left = true;
    }
    if (gpsComplete && right && left) {
        phase = 4;
        return;
    }
}

/*Landing用*/
bool isLanded() {
    bool isLanded = false;
    //フライトピンが加わったから，そのため書き直す
    if (Timer <= millis()) isLanded = true;
    else if (!isMoving()) isLanded = true;
    
    return isLanded;
}

/*GPSで方位と距離を計算する*/
double getGPS(double* direction, double* distance) {
    const unsigned long R = 6376008;//能代市役所から，地球の中心までの距離

    if (GPS.check() && GPS.isLocationUpdate()) {
        double dx, dy;//x, yの変位
        dx = R * (goal_longitude - (GPS.longitude() / 600000.0)) * cos(goal_latitude);
        dy = R * (goal_latitude - (GPS.latitude() / 600000.0));
        *direction = atan2(dy, dx);
        *distance = sqrt(pow(dx, 2) + pow(dy, 2));
        GPS.statusReset();
    }
}

/*その他*/
bool isMoving() {
    /*
    動いていたらtrue，動いていなかったらfalseを返す
    動いていないということは，すなわちスタックしている，ということではない．
    */
    const int thAccel[3] = {1, 2, 1};// 動いているかどうかの閾値設定(m/s^2)　添字0:x軸, 1:y軸，2:z軸
    bool isMoving = false;
    sensors_event_t accelData;
    bno.getEvent(&accelData);

    if ((accelData.acceleration.x > thAccel[0]) && (accelData.acceleration.y > thAccel[1]) && (accelData.acceleration.z > thAccel[2])) {
        isMoving = true;
    }
    return isMoving;
}

float Distance() {
    /*
    距離(cm)を返す
    一応，九軸センサー内蔵の温度センサーで温度を見て，校正しているが，だめっぽかったら15℃として計算してる
    これは, ambient temperature なのか， sensor temperatureなのか，よくわからん（データーシートにも混在）→実験してみる
    ！！メモリ節約のため，25度で計算
    */
    int Duration;
    float Distance;

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

/*Rotate to the ABSOLUTE angle (in euler)*/
void rotate(float angle) {
    /*旋回速度，実験で決定する*/
    const byte rotatePWM = 100;
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
    float angleNow = e_orientation_now.x();

    /*入れ替え
    float temp = q_orientation_now.x();
    q_orientation_now.x() = -q_orientation_now.y();
    q_orientation_now.y() = temp;
    q_orientation_now.z() = -q_orientation_now.z();
    //おそらく不要
    */
    /*
    imu::Vector<3> e_orientation_now = q_orientation_now.toEuler();
    float angleNow = radPI * e_orientation_now.z();
    */
    float destAngle = angle - angleNow;
    if (destAngle > 0) {
        while (angleNow <= destAngle) {
            /*右は後転，左は正転*/
            digitalWrite(PIN_MO_L_B, LOW);
            digitalWrite(PIN_MO_R_A, LOW);

            analogWrite(PIN_MO_L_A, rotatePWM);
            analogWrite(PIN_MO_R_B, rotatePWM);

            getRad(&e_orientation_now);
            angleNow = e_orientation_now.x();
        }
    } else if (destAngle < 0) {
        while (angleNow <= destAngle) {
            /*右は正転，左は後転*/
            digitalWrite(PIN_MO_L_A, LOW);
            digitalWrite(PIN_MO_R_B, LOW);

            analogWrite(PIN_MO_L_B, rotatePWM);
            analogWrite(PIN_MO_R_A, rotatePWM);

            getRad(&e_orientation_now);
            angleNow = e_orientation_now.x();
        }
    }
    motor_stop();
}

float getRad(imu::Vector<3>* e_orientation_now) {
    //止まっているとCalibrationが悪くなるので，だめだったらちょっと動かす用
    millisTemp = millis();

    uint8_t system;
    imu::Quaternion q_orientation_now;
    
    do {
        bno.getCalibration(&system, NULL, NULL, NULL);
        q_orientation_now = bno.getQuat();
        
        if (millisTemp + 1000 < millis()) {
            motor_foward(50, 50);
            delay(100);
            motor_stop();
        }
        
    } while (system < 1);//1, 2，3の時のみループを抜けて出力
    
    q_orientation_now.normalize();//重力分を取り除く（vector.h）
    *e_orientation_now = q_orientation_now.toEuler();
}

/*FOR DEBUG!!*/
/*ローカル変数のメモリもみてみる*/
/*
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
*/
