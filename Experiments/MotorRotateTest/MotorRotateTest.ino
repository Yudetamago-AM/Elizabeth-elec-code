#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <PinAssign.h>
#include <Motor.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
unsigned long millisTemp;

void setup() {
    /*serial (for debug) initialize*/
    Serial.begin(9600);
    Serial.println(F("Serial begin"));

    /*motor init*/
    motor_init();
    motor_stop();
    Serial.println("motor init done");

    /*BNO055 initialize*/
    while (!bno.begin()) {
        Serial.println(F("BNO055 not ready"));
        delay(100);
    }
    bno.setExtCrystalUse(true);//のっかってるからには使わねば（精度向上）
    Serial.println(F("BNO055 ready"));

    delay(2000);
    rotate(1);
}

void loop() {
}

/*Rotate to the ABSOLUTE angle (in euler)*/
void rotate(float angle) {
    motor_stop();//一応
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
    Serial.print("angleNow(at begin)");
    Serial.println(angleNow);

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
    if (destAngle > PI) destAngle -= PI;
    else if (destAngle < PI) destAngle += PI;
    
    Serial.print("destAngle: ");
    Serial.println(destAngle);
    
    if (destAngle > 0) {
            /*右は後転，左は正転*/
            digitalWrite(PIN_MO_L_B, LOW);
            digitalWrite(PIN_MO_R_A, LOW);

            analogWrite(PIN_MO_L_A, rotatePWM);
            analogWrite(PIN_MO_R_B, rotatePWM);
            
        while (angleNow <= destAngle) {
            getRad(&e_orientation_now);
            angleNow = e_orientation_now.x();
            
            Serial.print("angleNow 0ijou: ");
            Serial.println(angleNow);
            delay(20);
        }
        motor_stop();
        return;
    } else if (destAngle < 0) {
            /*右は正転，左は後転*/
            digitalWrite(PIN_MO_L_A, LOW);
            digitalWrite(PIN_MO_R_B, LOW);

            analogWrite(PIN_MO_L_B, rotatePWM);
            analogWrite(PIN_MO_R_A, rotatePWM);
            
        while (angleNow <= destAngle) {
            getRad(&e_orientation_now);
            angleNow = e_orientation_now.x();
            
            Serial.print("angleNow 0ika: ");
            Serial.println(angleNow);
            delay(20);
        }
        motor_stop();
        return;
    }
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
            motor_foward(100, 100);
            delay(200);
            motor_stop();
            motor_back(100, 100);
            delay(200);
            motor_stop();
        }
        
    } while (system < 0);//1, 2，3の時のみループを抜けて出力
    
    q_orientation_now.normalize();//重力分を取り除く（vector.h）
    *e_orientation_now = q_orientation_now.toEuler();
}
