#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PinAssign.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
    /*serial initialize*/
    Serial.begin(9600);
    Serial.println(F("serial begin"));

    /*BNO055 initialize*/
    while (!bno.begin()) {
        Serial.println("couldn't detect BNO055");
        delay(50);
    }
    bno.setExtCrystalUse(true);//のっかってるからには使わねば（精度向上）
}

void loop() {
    /*print temperature every seconds*/
    Serial.println(bno.getTemp());
    delay(1000);
}