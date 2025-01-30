#pragma once

#include <BLECharacteristic.h>
#include "config_manager.h"
#include <ArduinoJson.h>

// Callbacks para tiempo de sleep
class SleepConfigCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        uint32_t newTime = atoi(value.c_str());
        ConfigManager::setSleepTime(newTime);
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        uint32_t currentTime = ConfigManager::getSleepTime();
        pCharacteristic->setValue(String(currentTime).c_str());
    }
};

// Callbacks para NTC 100K
class NTC100KConfigCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, pCharacteristic->getValue());
        
        ConfigManager::setNTC100KConfig(
            doc["t1"] | 0.0,
            doc["r1"] | 0.0,
            doc["t2"] | 0.0,
            doc["r2"] | 0.0,
            doc["t3"] | 0.0,
            doc["r3"] | 0.0
        );
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        double t1, r1, t2, r2, t3, r3;
        ConfigManager::getNTC100KConfig(t1, r1, t2, r2, t3, r3);
        
        StaticJsonDocument<200> doc;
        doc["t1"] = t1;
        doc["r1"] = r1;
        doc["t2"] = t2;
        doc["r2"] = r2;
        doc["t3"] = t3;
        doc["r3"] = r3;
        
        String jsonString;
        serializeJson(doc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};

// Callbacks para Conductividad
class ConductivityConfigCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, pCharacteristic->getValue());
        
        ConfigManager::setConductivityConfig(
            doc["calTemp"] | 0.0f,
            doc["coefComp"] | 0.0f,
            doc["v1"] | 0.0f,
            doc["t1"] | 0.0f,
            doc["v2"] | 0.0f,
            doc["t2"] | 0.0f,
            doc["v3"] | 0.0f,
            doc["t3"] | 0.0f
        );
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        float calTemp, coefComp, v1, t1, v2, t2, v3, t3;
        ConfigManager::getConductivityConfig(calTemp, coefComp, v1, t1, v2, t2, v3, t3);
        
        StaticJsonDocument<200> doc;
        doc["calTemp"] = calTemp;
        doc["coefComp"] = coefComp;
        doc["v1"] = v1;
        doc["t1"] = t1;
        doc["v2"] = v2;
        doc["t2"] = t2;
        doc["v3"] = v3;
        doc["t3"] = t3;
        
        String jsonString;
        serializeJson(doc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};

// Callbacks para NTC 10K
class NTC10KConfigCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, pCharacteristic->getValue());
        
        ConfigManager::setNTC10KConfig(
            doc["t1"] | 0.0,
            doc["r1"] | 0.0,
            doc["t2"] | 0.0,
            doc["r2"] | 0.0,
            doc["t3"] | 0.0,
            doc["r3"] | 0.0
        );
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        double t1, r1, t2, r2, t3, r3;
        ConfigManager::getNTC10KConfig(t1, r1, t2, r2, t3, r3);
        
        StaticJsonDocument<200> doc;
        doc["t1"] = t1;
        doc["r1"] = r1;
        doc["t2"] = t2;
        doc["r2"] = r2;
        doc["t3"] = t3;
        doc["r3"] = r3;
        
        String jsonString;
        serializeJson(doc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};

// Callbacks para pH
class PHConfigCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, pCharacteristic->getValue());
        
        ConfigManager::setPHConfig(
            doc["v1"] | 0.0f,
            doc["t1"] | 0.0f,
            doc["v2"] | 0.0f,
            doc["t2"] | 0.0f,
            doc["v3"] | 0.0f,
            doc["t3"] | 0.0f,
            doc["calTemp"] | 25.0f
        );
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        float v1, t1, v2, t2, v3, t3, calTemp;
        ConfigManager::getPHConfig(v1, t1, v2, t2, v3, t3, calTemp);
        
        StaticJsonDocument<200> doc;
        doc["v1"] = v1;
        doc["t1"] = t1;
        doc["v2"] = v2;
        doc["t2"] = t2;
        doc["v3"] = v3;
        doc["t3"] = t3;
        doc["calTemp"] = calTemp;
        
        String jsonString;
        serializeJson(doc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};

class SensorsConfigCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        StaticJsonDocument<512> doc;
        deserializeJson(doc, pCharacteristic->getValue());
        
        const char* sensorId = doc["sensorId"] | "";
        const char* sensorName = doc["sensorName"] | "";
        bool enable = doc["enable"] | false;
        const char* tempSensorId = doc["tempSensorId"] | "";
        
        ConfigManager::setSensorConfig(sensorId, sensorName, enable, tempSensorId);
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        DynamicJsonDocument doc(1024);
        JsonArray sensorArray = doc.createNestedArray("sensors");
        
        std::vector<SensorConfig> configs = ConfigManager::getAllSensorConfigs();
        
        for (const auto& sensor : configs) {
            JsonObject sensorObj = sensorArray.createNestedObject();
            sensorObj["sensorId"] = sensor.sensorId;
            sensorObj["sensorName"] = sensor.sensorName;
            sensorObj["type"] = (int)sensor.type;
            sensorObj["adcNumber"] = sensor.adcNumber;
            sensorObj["channel"] = sensor.channel;
            sensorObj["tempSensorId"] = sensor.tempSensorId;
            sensorObj["enable"] = sensor.enable;
        }
        
        String jsonString;
        serializeJson(doc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};