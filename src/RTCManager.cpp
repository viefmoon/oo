#include "RTCManager.h"

RTCManager::RTCManager() {}

bool RTCManager::begin() {
    return rtc.begin();
}

void RTCManager::setFallbackDateTime() {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("RTC configurado con hora de compilación");
}

DateTime RTCManager::getCurrentTime() {
    return rtc.now();
}

void RTCManager::printDateTime() {
    DateTime currentTime = getCurrentTime();
    
    Serial.print(currentTime.year(), DEC);
    Serial.print('/');
    Serial.print(currentTime.month(), DEC);
    Serial.print('/');
    Serial.print(currentTime.day(), DEC);
    Serial.print(" ");
    Serial.print(currentTime.hour(), DEC);
    Serial.print(':');
    Serial.print(currentTime.minute(), DEC);
    Serial.print(':');
    Serial.println(currentTime.second(), DEC);
}

uint32_t RTCManager::getEpochTime() {
    DateTime now = getCurrentTime();
    return now.unixtime();
}
