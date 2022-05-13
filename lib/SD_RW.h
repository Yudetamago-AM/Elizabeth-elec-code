#ifndef SD_RW_H
#define SD_RW_H

#include <Arduino.h>
#include <PinAssign.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <bcdtime.h>

class SD_RW {
    public:
        SD_RW();

        void init();
        void log(String text);
        void gpsLog(bcdtime_t bcdtime, int32_t longitude, int32_t latitude, int32_t speed, float angle);

    private:
        String fileName;
        int countFileName;
        int countlog = 0;
        int countgpslog = 0;
};

#endif