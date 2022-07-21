/*
ゴールであるカラーコーンの座標を取得してシリアルに出力する．
これで取得した緯度経度などを，Main.inoのほうでつかう．
ほとんどライブラリのサンプルコード
*/

#include <Arduino.h>
#include <Wire.h>
#include <PinAssign.h>
//#include <GPS_MTK333X_Serial.h>
#include <SoftwareSerial.h>
#include <GPS_MTK333X_SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/*以下lib内自作ライブラリ*/
#include <PinAssign.h>
#include <Motor.h>
#include <SD_RW.h>

GPS_MTK333X_SoftwareSerial GPS(PIN_GPS_RX, PIN_GPS_TX);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
/*重要！！ゴールのGPS座標*/
const long goal_longitude = 81462106; //経度*600000(60万)
const long goal_latitude = 20879821; //緯度*600000//知真館3号館
unsigned long millisTemp = 0;
unsigned long millisTemp2 = 0;
bool isFirstUnplug = true;
byte goalCount = 0;

void setup() {
    Serial.begin(9600);
    Serial.println(F("serial begin"));

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
    Serial.println(F("GPS ready"));

    /*BNO055 initialize*/
    while (!bno.begin()) {
        Serial.println(F("BNO055 not ready"));
        delay(100);
    }
    bno.setExtCrystalUse(true);//のっかってるからには使わねば（精度向上）
    Serial.println(F("BNO055 ready"));

}

void loop() {
    /*
    if (GPS.check() && GPS.isTimeUpdate()) {
        bool f = GPS.isLocationUpdate();
        GPSInfo_t gpsInfo = GPS.value();
        Serial.println("gpsInfo.date(BCD date):");
        Serial.println(gpsInfo.date, HEX);
        Serial.println("gpsInfo.date(BCD time):");
        Serial.println(gpsInfo.time, HEX);
        if (f) {
            Serial.println("gpsInfo.satellites(how many):");
            Serial.println(gpsInfo.satellites, DEC);
            Serial.println("gpsInfo.dop(bigger means accurate)(DOP):");
            Serial.println(gpsInfo.dop / 100.0);
            
            //小数点以下7桁で1cm精度
            
            Serial.println("gpsInfo.latitude(ido)(degree):");
            Serial.println(gpsInfo.latitude);
            Serial.println("gpsInfo.longtitude(keido)(degree):");
            Serial.println(gpsInfo.longitude);
            Serial.println("gpsInfo.altitude(koudo)(m):");
            Serial.println(gpsInfo.altitude / 100.0);
        }
        Serial.println();
    }
    */
    double direction, distance;
    getGPS(&direction, &distance);
    Serial.println("direction: " + String(direction) + "(rad)");
    sd_log("dir: " + String(direction));
    Serial.println("distance:" + String(distance) + "(cm)");
    sd_log("dist: " + String(distance));
}

/*GPSで方位と距離を計算する*/
double getGPS(double* direction, double* distance) {
    const unsigned long R = 6376008;//能代市役所から，地球の中心までの距離
    imu::Vector<3> e_orientation_now;
    getRad(&e_orientation_now);

    //4debug
    //uint32_t start, end;

    while (true) {
        if (GPS.check() && GPS.isTimeUpdate() && GPS.isLocationUpdate()) {
        //Serial.println(F("gps updated (getGPS)"));
        //byte tri_goal_latitude = underbyte(goal_latitude);

        //計算
        
        /*
        dx = R * (goal_longitude - GPS.longitude());// * long(cos(goal_latitude / 600000.0) * 100);
        Serial.println(goal_longitude - GPS.longitude());
        Serial.print(F("dx: "));
        Serial.println(dx);
        
        dy = R * (goal_latitude - GPS.latitude());// * 100;
        Serial.println(goal_latitude - GPS.latitude());
        Serial.print(F("dy: "));
        Serial.println(dy);
        */
        Serial.print("long: ");
        Serial.println(GPS.longitude());
        Serial.print("lat: ");
        Serial.println(GPS.latitude());
        //x, yの変位
        long dx = ((goal_longitude - GPS.longitude()) * 153);//0.000001度で0.92m(京田辺)，0.85m(能代)より，単位メートル
        long dy = ((goal_latitude - GPS.latitude()) * 185);//0.000001度で0.111m(111)より0.1
        
        if (dx == 0 && dy == 0) *direction = 0;
        else *direction = atan2(dx, dy);//意図的にdx, dyの順，というのも，北基準だから．
        *distance = approx_distance(dx, dy) / 10;//単位:cm
        //Serial.println(F("calc done (getGPS)"));
        //Serial.print(F("dir: "));
        //Serial.println(*direction);
        //Serial.print(F("dist: "));
        //Serial.println(*distance);

        //4debug
        //start = millis();
        //sd_gpslog(GPS.longitude(), GPS.latitude(), e_orientation_now.x());
        //sd_gpslog(GPS.longitude() / 600000.0, GPS.latitude() / 600000.0, e_orientation_now.x());
        //end = millis() - start;

        //Serial.print(F("sd write took(ms) : "));//およそ95msかかっていた
        //Serial.println(end);
        break;
        }
    }
    
    Serial.println(F("gps logged!"));

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

float getRad(imu::Vector<3>* e_orientation_now) {
    //止まっているとCalibrationが悪くなるので，だめだったらちょっと動かす用
    millisTemp = millis();

    uint8_t mag;//磁気センサーのキャリブレーション度を見る．
    imu::Quaternion q_orientation_now;
    /*
    do {
        bno.getCalibration(NULL, NULL, NULL, &mag);//system, gyro, accel, mag

        //for debug
        
        Serial.print("mag: ");
        Serial.println(mag);
        
        q_orientation_now = bno.getQuat();
        
        
        if (millisTemp + 1000 < millis()) {
            motor_rotate_R(100);
            delay(200);
            motor_rotate_L(100);
            delay(200);
            motor_stop();
        }
        
    } while (mag < 1);//1, 2，3の時のみループを抜けて出力
    //一時的にすべての場合にしてみる
    */
    
    q_orientation_now.normalize();//重力分を取り除く（vector.h）
    *e_orientation_now = q_orientation_now.toEuler();
}