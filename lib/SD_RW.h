#ifndef SD_RW
#define SD_RW

#include <Arduino.h>
#include <PinAssign.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

class SD_RW {
    public:
        SD_RW();

        void init();
        void write(String text);
    private:
        String fileName;
};

#endif