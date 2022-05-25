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
void sd_gpslog(bcdtime_t bcdtime, int32_t longitude, int32_t latitude, int32_t speed, float angle);

#endif