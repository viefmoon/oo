#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#include <Arduino.h>
#include <RTClib.h>
#include <Wire.h>

class RTCManager {
private:
    RTC_DS3231 rtc;

public:
    RTCManager();
    bool begin();
    void setFallbackDateTime();
    DateTime getCurrentTime();
    void printDateTime();
    bool hasLostPower();
    uint32_t getEpochTime();
};

#endif 