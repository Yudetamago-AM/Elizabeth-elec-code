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
#include <SD_RW.h>

//ゴールの経緯度，とりあえず奈良市役所にしている
const double goal_longitude = 135.80472222; //経度
const double goal_latitude = 34.68499999; //緯度
unsigned long millisTemp = 0;

GPS_MTK333X_SoftwareSerial GPS(PIN_GPS_RX, PIN_GPS_TX);
//GPS_MTK333X_Serial GPS;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
    /*serial (for debug) initialize*/
    Serial.begin(9600);
    Serial.println(F("Serial begin"));

    /*sd init*/
    sd_init();

    /*gps initialize*/
    while (!GPS.begin(9600)) {//9600
        Serial.println(F("GPS not ready"));
        delay(100);
    }
    // UARTボーレートを 9600bpsに設定する
    GPS.sendMTKcommand(251, F(",9600"));
    // 1000ms間隔で NMEAを出力する
    GPS.sendMTKcommand(220, F(",200"));//1000msごとで十分かは要検証→明らかに不十分であった
    //GPS.sendMTKcommand(300, F(",200,0,0,0,0"));
    // AlwaysLocate モード開始 
    //GPS.sendMTKcommand(225, F(",8"));
    // 上記を解除してスタンダードモードに遷移
    GPS.sendMTKcommand(225, F(",0"));
    // QZSS（みちびき）をサポートする
    GPS.sendMTKcommand(351, F(",1"));
    // RMCとGGAをともに1サイクルで出力する（1サイクル時間は PMTK220 による）
    // 各項は ",GLL,RMC,VTG,GGA,GSA,GSV,0,0,0,0,0,0,0,0,0,0,0,ZDA,MCHN"
    GPS.sendMTKcommand(314, F(",0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"));

    /*BNO055 initialize*/
    while (!bno.begin()) {
        Serial.println(F("BNO055 not ready"));
        delay(100);
    }
    bno.setExtCrystalUse(true);//のっかってるからには使わねば（精度向上）
    Serial.println(F("BNO055 ready"));

    byte i;
    for (i = 0; i < 5; i++) {
        //書き込み
        sd_log("hello sd card!>" + String(i));//ここはなぜかFオプションが効かない
        delay(10);
    }

    Serial.println(F("log done!"));
}

void loop(){
    imu::Vector<3> e_orientation_now;
    /*
    getGPS(&direction, &distance);
    delay(50);
    Serial.print(F("direction: "));
    Serial.println(direction);
    Serial.print(F("distance: "));
    Serial.println(distance);
    */

    //ログ
    if (GPS.check() && GPS.isLocationUpdate() && GPS.isTimeUpdate()) {
        /*
        getRad(&e_orientation_now);
        Serial.print(F("x orientation: "));
        Serial.println(e_orientation_now.x());
        */
        //sd_log("G: " + String(GPS.time()));
        //delay(20);
        sd_gpslog(String(GPS.longitude() / 600000.0, 7), String(GPS.latitude() / 600000.0, 7), String(1));
        delay(50);
        Serial.println(F("gps logged!"));
    }
    delay(500);
    
}

/*GPSで方位と距離を計算する
//Main.inoより
double getGPS(double* direction, double* distance) {
    const unsigned long R = 6376008;//能代市役所から，地球の中心までの距離
    imu::Vector<3> e_orientation_now;

    GPS.statusReset();
    if (GPS.check() && GPS.isLocationUpdate()) {
        double dx, dy;//x, yの変位
        Serial.println(F("gps updated (getGPS)"));

        //計算
        dx = R * (goal_longitude - (GPS.longitude() / 600000.0)) * cos(goal_latitude);
        dy = R * (goal_latitude - (GPS.latitude() / 600000.0));
        *direction = atan2(dy, dx);
        *distance = sqrt(pow(dx, 2) + pow(dy, 2));
        Serial.println(F("calc done (getGPS)"));
    }
}
*/

double getRad(imu::Vector<3>* e_orientation_now) {
    //止まっているとCalibrationが悪くなるので，だめだったらちょっと動かす用
    //millisTemp = millis();

    uint8_t system;
    imu::Quaternion q_orientation_now;
    
    do {
        bno.getCalibration(&system, NULL, NULL, NULL);
        q_orientation_now = bno.getQuat();
        /*
        if (millisTemp + 1000 < millis()) {
            motor_foward(50, 50);
            delay(100);
            motor_stop();
        }
        */
        
    } while (system < 1);//1, 2，3の時のみループを抜けて出力
    
    q_orientation_now.normalize();//重力分を取り除く（vector.h）
    *e_orientation_now = q_orientation_now.toEuler();
}