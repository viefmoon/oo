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
        DeserializationError error = deserializeJson(doc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando NTC100K config: "));
            Serial.println(error.c_str());
            return;
        }
        
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
        DeserializationError error = deserializeJson(doc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando Conductivity config: "));
            Serial.println(error.c_str());
            return;
        }
        
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
        DeserializationError error = deserializeJson(doc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando NTC10K config: "));
            Serial.println(error.c_str());
            return;
        }
        
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
        DeserializationError error = deserializeJson(doc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando pH config: "));
            Serial.println(error.c_str());
            return;
        }
        
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
        DynamicJsonDocument doc(2048);
        DeserializationError error = deserializeJson(doc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando Sensors config: "));
            Serial.println(error.c_str());
            return;
        }
        
        std::vector<SensorConfig> configs;
        JsonArray sensorArray = doc["sensors"];
        
        for (JsonVariant sensor : sensorArray) {
            SensorConfig config;
            strncpy(config.configKey, sensor["k"] | "", sizeof(config.configKey));
            strncpy(config.sensorId, sensor["id"] | "", sizeof(config.sensorId));
            strncpy(config.tempSensorId, sensor["ts"] | "", sizeof(config.tempSensorId));
            config.type = static_cast<SensorType>(sensor["t"] | 0);
            config.enable = sensor["e"] | false;
            
            configs.push_back(config);
        }
        
        ConfigManager::setSensorsConfigs(configs);
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        DynamicJsonDocument doc(2048);
        JsonArray sensorArray = doc.createNestedArray("sensors");
        
        std::vector<SensorConfig> configs = ConfigManager::getAllSensorConfigs();
        
        for (const auto& sensor : configs) {
            JsonObject obj = sensorArray.createNestedObject();
            obj["k"] = sensor.configKey;
            obj["id"] = sensor.sensorId;
            obj["t"] = static_cast<int>(sensor.type);
            obj["ts"] = sensor.tempSensorId;
            obj["e"] = sensor.enable;
        }
        
        String jsonString;
        serializeJson(doc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};

class LoRaConfigCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando LoRa config: "));
            Serial.println(error.c_str());
            return;
        }
        
        // Extraer la configuración desde el JSON recibido
        uint32_t devAddr = doc["devAddr"] | 0;
        String fNwkSIntKey = doc["fNwkSIntKey"] | "";
        String sNwkSIntKey = doc["sNwkSIntKey"] | "";
        String nwkSEncKey  = doc["nwkSEncKey"]  | "";
        String appSKey     = doc["appSKey"]     | "";
        
        // Actualizar la configuración LoRa usando ConfigManager
        ConfigManager::setLoRaConfig(devAddr, fNwkSIntKey, sNwkSIntKey, nwkSEncKey, appSKey);
    }
    
    void onRead(BLECharacteristic* pCharacteristic) override {
        // Obtener la configuración LoRa almacenada
        LoRaConfig config = ConfigManager::getLoRaConfig();
        
        StaticJsonDocument<256> doc;
        doc["devAddr"]       = config.devAddr;
        doc["fNwkSIntKey"]   = config.fNwkSIntKey;
        doc["sNwkSIntKey"]   = config.sNwkSIntKey;
        doc["nwkSEncKey"]    = config.nwkSEncKey;
        doc["appSKey"]       = config.appSKey;
        
        String jsonString;
        serializeJson(doc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};

class DeviceIdConfigCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        ConfigManager::setDeviceId(String(value.c_str()));
        Serial.print("Nuevo deviceId configurado: ");
        Serial.println(value.c_str());
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        String currentId = ConfigManager::getDeviceId();
        pCharacteristic->setValue(currentId.c_str());
    }
};