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

#define ANGLE_TH 0.39//前45度，GPS誘導
#define LONGITUDE_PER_RES 141//能代
#define LATITUDE_PER_RES 185

byte phase = 0;
/*重要！！電源オンからのタイマー，ミリ秒単位で指定*/
const unsigned long Timer = 600000; //600 * 1000, 600秒 は 10分
/*フライトピン解放からのタイマー*/
const unsigned int flightPinTimer = 10000;//50s//10s
unsigned long flightPinMillis;
/*重要！！ゴールのGPS座標*/
const long goal_longitude = 81462526; //経度*600000(60万)
const long goal_latitude = 20880923; //緯度*600000

bool isFirstUnplug = true;
bool isFirstDIST = false;
byte goalCount = 0;

GPS_MTK333X_SoftwareSerial GPS(PIN_GPS_RX, PIN_GPS_TX);
//GPS_MTK333X_Serial GPS;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
    /*Motor initialize*/
    motor_init();
    motor_stop();

    /*ニクロム線の初期設定*/
    pinMode(PIN_NICHROME, OUTPUT);
    digitalWrite(PIN_NICHROME, LOW);

    /*//Serial (for debug) initialize*/
    Serial.begin(9600);
    Serial.println(F("Serial begin"));

    /*FOR DEBUG!! mem check*/
    /*不要なときはコメントアウト！！*/
    // 5/16時点で723bytesあまってる感じ
    /*
    //Serial.print(F("Free memory="));
    //Serial.print(freeRam(), DEC);
    //Serial.println(F("[bytes]"));
    */

    /*SD initialize*/
    sd_init();

    /*BNO055 initialize*/
    while (!bno.begin()) {
        //Serial.println(F("BNO055 not ready"));
        delay(100);
    }
    bno.setExtCrystalUse(true);//のっかってるからには使わねば（精度向上）
    //Serial.println(F("BNO055 ready"));

    /*GPS initialize*/
    while (!GPS.begin(9600)) {
        //Serial.println(F("GPS not ready"));
        delay(100);
    }
    // UARTボーレートを 9600bpsに設定する
    GPS.sendMTKcommand(251, F(",9600"));
    // 1000ms間隔で NMEAを出力する
    GPS.sendMTKcommand(220, F(",1000"));
    //GPS.sendMTKcommand(300, F(",1000,0,0,0,0"));
    // AlwaysLocate モード開始 
    //GPS.sendMTKcommand(225, F(",8"));
    // 上記を解除してスタンダードモードに遷移
    GPS.sendMTKcommand(225, F(",0"));
    // QZSS（みちびき）をサポートする
    GPS.sendMTKcommand(351, F(",1"));
    // RMCとGGAをともに1サイクルで出力する（1サイクル時間は PMTK220 による）
    // 各項は ",GLL,RMC,VTG,GGA,GSA,GSV,0,0,0,0,0,0,0,0,0,0,0,ZDA,MCHN"
    GPS.sendMTKcommand(314, F(",0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"));
    //Serial.println(F("GPS ready"));

    /*Distance sensor initialize*/
    pinMode(PIN_DIST_TRIG, OUTPUT);
    pinMode(PIN_DIST_ECHO, INPUT);
    digitalWrite(PIN_DIST_TRIG, LOW);

    /*flight pin initialize*/
    pinMode(PIN_FLIGHT, INPUT);

    //4debug
    //Serial.println(F("setup done!"));
    sd_log(F("State,Decision,Control"));//S: State, D: Decision, C: Control, R: Result
    sd_log(F("Init done!"));
    sd_log("G_lat:" + String(goal_latitude));
    sd_log("G_long:" + String(goal_longitude));
    //getGPS(NULL, NULL);
    delay(50);
}

void loop() {
    //GPS受信バッファあふれ防止→たぶん不要？（ちゃんと見に行くから）
    //GPS.check();
    switch (phase) {
    case 0://パラシュート展開まで待つ
        waitTilUnPlug();
        break;
    case 1:
        Landing();
        break;
    case 2:
        GuideGPS();
        //stack();
        break;
    case 3:
        //Backward();
        GuideDIST();
        //stack();
        break;
    case 4:
        //Backward();
        Goal();
        //stack();
        break;
    case 5://ゴール後
        GPS.check();
        delay(10000);
        break;
    }
}

/*各シークエンス*/
void waitTilUnPlug() {
    Serial.println(F("wait"));
    //フライトピン
    if ((digitalRead(PIN_FLIGHT) == HIGH) && isFirstUnplug) {
        flightPinMillis = millis();
        isFirstUnplug = false;
        //Serial.println(F("pin ms set!"));
        sd_log(F("F_Pin:removed"));
        phase = 1;//landing
        BnoGetCal();
    } else if (Timer <= millis()) {
        phase = 1;
    }
    delay(50);
    return;
}

void Landing() {
    Serial.println(F("land"));
    //Serial.println(F("Landing seq begin"));
    //if (millis() <= flightPinMillis + flightPinTimer) return;
    if (((flightPinMillis + flightPinTimer) <= millis())) {
        sd_log(F(",F_Pin:landed"));
        // 前にうごかしたりパラシュート切り離したり…
        /*ニクロム線のカット*/
        Nichrome();
    } else if (Timer <= millis()){
        sd_log(F(",Timer:landed"));
        // 前にうごかしたりパラシュート切り離したり…
        /*ニクロム線のカット*/
        Nichrome();
    }

    BnoGetCal();
    //Serial.println(F("done"));
}

void BnoGetCal() {
    //センサーのキャリブレーションしとく
    byte mag;
    for (byte i = 0; i < 100; i++) {
        bno.getCalibration(NULL, NULL, NULL, &mag);
        delay(10);
        if (mag > 0) break;
    }
    return;
}

void Nichrome(){
        Serial.println(F("nic"));
        digitalWrite(PIN_NICHROME, HIGH);
        delay(2000);
        motor_forward(255, 255);
        delay(3000);
        digitalWrite(PIN_NICHROME,LOW);
        motor_stop();
        sd_log(F(",,Nichrome heat"));
        phase = 2;//gps
}

/*guide to the goal using GPS*/
void GuideGPS() {
    Serial.println(F("gps"));
    //Serial.println(F("gidG bgn"));
    sd_log(F("GPS Guide"));
    //方位・距離を計算
    double direction, distance, angle;//方位，距離，角度の差
    imu::Vector<3> e_orientation_now;

    getGPS(&direction, &distance);
    sd_log("dir:" + String(direction));
    sd_log("dist:" + String(distance));

    //ゴールまでgpsで2.5m以内なら，精密誘導へ
    if (distance <= 300) {//1.5 * 100 * 600000，あってるかわからない
        motor_stop();
        phase = 3;//guideDIST
        sd_log(F(",GPS Guide done"));
        return;
    }
    
    //if (distance <= 600) ANGLE_TH = 0.2;//左右各12度，全部で24度
    //方位をみて制御
    getRad(&e_orientation_now);
    //angle = e_orientation_now.x() - direction;

    //bool isPlus = false; //trueなら正，falseなら負
    //if (angle > 0) isPlus = true;
    
    //if (direction > 0) angle = direction + e_orientation_now.x();
    //else angle = direction - e_orientation_now.x();
    
    angle = direction + e_orientation_now.x();

    while (angle > PI_FLOAT) {
        angle -= PI_FLOAT;
    }
    while (angle < ((-1) * PI_FLOAT)) {
        angle += PI_FLOAT;
    }
    /*
    sd_log(",goal or:" + String(angle) + ",,");

    if (angle <= 0.26 && angle >= -0.26) {
        motor_forward(255, 255);//だいたい15度, 3.15-0.26
        sd_log(",,m_f:2525,");
    } else if (angle <= 1.3) {
        motor_forward(178, 255);//だいたい75度，ゴールが右手，出70%くらい
        sd_log(",,m_f:1725");
    } else if (angle >= -1.3) {
        motor_forward(255, 178);
        sd_log(",,m_f:2517");
    } else if (angle >= 0) {
        motor_forward(76, 255);
        sd_log(",,m_f:0725");
    } else if (angle <= 0) {
        motor_forward(255, 76);//わざと3.15，というのも，PI = 3.141592...で，3.14ならカバーできない角度が生まれるため
        sd_log(",,m_f:2507");
    }
    */

    //パターン2
    if ((angle > ((-1) * ANGLE_TH)) && (angle < ANGLE_TH)) {
        motor_forward(255, 255);
        sd_log(",diff:" + String(angle));
        sd_log(F(",,m_f:255255"));
        delay(900);
    } else {
        motor_rotate_L(255);
        delay(200);
        motor_stop();
        rotate((-1) * direction);
        sd_log(",rotate to:" + String((-1) * direction));
        motor_stop();
    }
    //delay(800);

    //delay(2000);
    //delayしなくても，1Hzくらいではうごくはず．→うごいた

    //Serial.println(F("done"));
    return;
}

void GuideDIST() {
    //Serial.println(F("gidD bgn"));
    if (isFirstDIST == true) {
        motor_forward(255, 255);
        delay(2000);
        motor_stop();
        isFirstDIST = false;
    }
    /*精密誘導が上手くいかない場合*/
    if (goalCount > 4) {
        phase = 5;
        return;
    }

    sd_log(F("DIST Guide"));
    imu::Vector<3> e_orientation_now;
    getRad(&e_orientation_now);

    byte dist[18], i;//あるいは，印付けてみるとか？
    dist[0] = 200;
    rotate(e_orientation_now.x() - 1.57);//90度左回転
    
    for (i = 0; i < 18; i++) {
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

    for (i = 0; i < 18; i++) {
        if((dist[i] < 200) && (abs(dist[i] - dist[i - 1]) < 10)) {
            //ログ
            sd_log("dist_num:" + String(i));//distance direction
            sd_log("dist:" + String(dist[i]));

            //制御
            getRad(&e_orientation_now);
            rotate(e_orientation_now.x() - (0.175 * (18 - i)));
            delay(10);
            //millisTemp2 = millis();
            sd_log(F(",,m_f:255255"));
            motor_forward(255, 255);
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
    sd_log(F("isGoal"));

    bool gpsComplete = false;
    bool right = false, left = false;
    double direction, distance;

    getGPS(&direction, &distance);
    if (distance < 400) {//gpsで5m以内
        gpsComplete = true;
    } else if (goalCount > 4) {
        phase = 2;
        return;
    }

    //0m判定
    if (gpsComplete && (Distance() < 5)) {
        motor_rotate_R(100);
        delay(200);
        motor_stop();
        if (Distance() < 5) right = true;

        motor_rotate_L(100);
        delay(200);
        motor_stop();
        if (Distance() < 5) left = true;
    } else {
        phase = 4;
    }
    if (gpsComplete && right && left) {
        phase = 5;//guide done -- wait for power off
        //Serial.println(F("0m goal!"));
    }
    goalCount++;
    return;
}

void Backward() {
    imu::Vector<3> e_orientation_now;
    getRad(&e_orientation_now);
    sd_log("or_y:" + String(e_orientation_now.y()));
    sd_log(F(",,m_f:178178"));

    while (!((e_orientation_now.z() > -0.3) && (e_orientation_now.z() < 0.3))) {//あとで閾値設定  
        motor_forward(178, 178);
        getRad(&e_orientation_now);
    }
    motor_stop();
    
    return;
}

/*Landing用
bool isLanded() {
    bool isLanded = false;
    //フライトピンが加わったから，そのため書き直す
    if (Timer <= millis()) isLanded = true;
    else if (!isMoving()) isLanded = true;
    
    return isLanded;
}
*/

/*GPSで方位と距離を計算する
double getGPS(double* direction, double* distance, imu::Vector<3>* e_orientation_now) {
    const unsigned long R = 6376008;//能代市役所から，地球の中心までの距離

    if (GPS.check() && GPS.isLocationUpdate()) {
        double dx, dy;//x, yの変位
        double longitude = GPS.longitude();
        double latitude = GPS.latitude();
        //計算
        dx = R * (goal_longitude - (longitude / 600000.0)) * cos(goal_latitude);
        dy = R * (goal_latitude - (latitude / 600000.0));
        *direction = atan2(dy, dx);
        *distance = sqrt(pow(dx, 2) + pow(dy, 2));
        //ログ
        sd_gpslog(GPS.time(), &longitude, &latitude, e_orientation_now.x())
        GPS.statusReset();
    }
}
*/
/*GPSで方位と距離を計算する*/
double getGPS(double* direction, double* distance) {
    //const unsigned long R = 6376008;//能代市役所から，地球の中心までの距離
    unsigned long millisTemp;
    imu::Vector<3> e_orientation_now;

    //4debug
    //uint32_t start, end;

    while (true) {
        if (GPS.check() && GPS.isTimeUpdate() && GPS.isLocationUpdate()) {
        //Serial.println(F("gps updated (getGPS)"));
        //byte tri_goal_latitude = underbyte(goal_latitude);
        getRad(&e_orientation_now);
        sd_gpslog(String(GPS.longitude()), String(GPS.latitude()), String(e_orientation_now.x(), 4));

        //計算
        
        /*
        dx = R * (goal_longitude - GPS.longitude());// * long(cos(goal_latitude / 600000.0) * 100);
        //Serial.println(goal_longitude - GPS.longitude());
        //Serial.print(F("dx: "));
        //Serial.println(dx);
        
        dy = R * (goal_latitude - GPS.latitude());// * 100;
        //Serial.println(goal_latitude - GPS.latitude());
        //Serial.print(F("dy: "));
        //Serial.println(dy);
        */
        //x, yの変位
        long dx = ((goal_longitude - GPS.longitude()) * LONGITUDE_PER_RES);//0.000001度で0.092m(京田辺)，0.085m(能代)より，単位メートル
        long dy = ((goal_latitude - GPS.latitude()) * LATITUDE_PER_RES);//0.000001度で0.111m(111)より0.1
        
        if (dx == 0 && dy == 0) *direction = 0;
        else *direction = atan2(dx, dy);//意図的にdx, dyの順，というのも，北基準だから．
        *distance = approx_distance(dx, dy) / 10;//単位:cm
        //Serial.println(F("calc done (getGPS)"));
        //Serial.print(F("dir: "));
        //Serial.println(*direction);
        //Serial.print(F("dist: "));
        //Serial.println(*distance);

        //millisTemp = millis();
        //sd_log("dir:" + String(*direction));
        //while ((millisTemp + 5) < millis()) {}
        //sd_log("dist:" + String(*distance));

        //4debug
        //start = millis();
        //sd_log(",,," + String(GPS.longitude()) + "," + String(GPS.latitude()) + "," + String(e_orientation_now.x()));
        //sd_gpslog(String(GPS.longitude()), String(GPS.latitude()), String(e_orientation_now.x(), 4));
        //sd_gpslog(GPS.longitude() / 600000.0, GPS.latitude() / 600000.0, e_orientation_now.x());        
        //end = millis() - start;

        //Serial.print(F("sd write took(ms) : "));//およそ95msかかっていた
        //Serial.println(end);
        break;
        }
    }
    
    //Serial.println(F("gps logged!"));

}

/*平方根を使わず2点間の距離を近似*/
//参考
//https://nowokay.hatenablog.com/entry/20120604/1338773843
//https://dora.bk.tsukuba.ac.jp/~takeuchi/?%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%9F%E3%83%B3%E3%82%B0%2F%E5%B9%B3%E6%96%B9%E6%A0%B9%E3%82%92%E4%BD%BF%E3%82%8F%E3%81%9A%E3%81%AB%E8%B7%9D%E9%9B%A2%E3%82%92%E6%B1%82%E3%82%81%E3%82%8B
long approx_distance(long dx, long dy) {
   unsigned long min, max, approx;

   if (dx < 0) dx = -dx;
   if (dy < 0) dy = -dy;

   if (dx < dy) {
      min = dx;
      max = dy;
   } else {
      min = dy;
      max = dx;
   }

   approx = (max * 983) + (min * 407);
   if (max < (min << 4)) approx -= ( max * 40 );

   // add 512 for proper rounding
   return ((approx + 512) >> 10);
} 

/*その他
bool isMoving() {
    
    //動いていたらtrue，動いていなかったらfalseを返す
    //動いていないということは，すなわちスタックしている，ということではない．
    const int thAccel[3] = {1, 2, 1};// 動いているかどうかの閾値設定(m/s^2)　添字0:x軸, 1:y軸，2:z軸
    bool isMoving = false;
    sensors_event_t accelData;
    bno.getEvent(&accelData);

    if ((accelData.acceleration.x > thAccel[0]) && (accelData.acceleration.y > thAccel[1]) && (accelData.acceleration.z > thAccel[2])) {
        isMoving = true;
    }
    return isMoving;
}
*/

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

/*Rotate to the ABSOLUTE angle (in euler)*/
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
            sd_log(F(",,m_r_L:255"));
            
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
            sd_log(F(",,m_r_R:255"));
            
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
/*
void stack() {
    if ((phase > 1) && isMoving) {
        //あとでちゃんと動くか確かめる
        imu::Vector<3> e_orientation_now;
        
        getRad(&e_orientation_now);
        while (e_orientation_now.y() < 0) {
            if (e_orientation_now.y() > 0.3) break;
            motor_forward(255, 255);
        }
        motor_stop();
    }
    
}
*/

/*FOR DEBUG!!*/
/*ローカル変数のメモリもみてみる*/
/*
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
*/
