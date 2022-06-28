#ifndef SD_RW_H
#define SD_RW_H

#include <Arduino.h>
#include <PinAssign.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <bcdtime.h>

/*initialize sd card(make file, etc)*/
void sd_init();

/*State,Decision,Control,Result*/
void sd_log(String text);

/*log location, angle*/
//void sd_gpslog(long longitude, long latitude, float angle);

#endif