#ifndef SD_RW_H
#define SD_RW_H

#include <Arduino.h>
#include <PinAssign.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <bcdtime.h>

void sd_init();
void sd_log(String text);
void sd_gpslog(String longitude, String latitude, String angle);

#endif