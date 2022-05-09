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
        void write(String text);
        String SD_GetDirName();

    private:
        String fileName;
        int countFileName;
};

#endif