#pragma once

#include <BLECharacteristic.h>
#include "config_manager.h"
#include <ArduinoJson.h>
#include "config.h"

// Callback unificada para la configuración del sistema (system, sleep y device)
class SystemConfigCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        // Se espera un JSON de la forma: { "system": { "initialized": <bool>, "sleep_time": <valor>, "device_id": "<valor>" } }
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando System config: "));
            Serial.println(error.c_str());
            return;
        }
        JsonObject obj = doc[NAMESPACE_SYSTEM];
        bool initialized = obj[KEY_INITIALIZED] | false;
        uint32_t sleepTime = obj[KEY_SLEEP_TIME] | DEFAULT_TIME_TO_SLEEP;
        String deviceId = obj[KEY_DEVICE_ID] | "";
        String stationId = obj[KEY_STATION_ID] | "";
        ConfigManager::setSystemConfig(initialized, sleepTime, deviceId, stationId);

    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        bool initialized;
        uint32_t sleepTime;
        String deviceId;
        String stationId;
        ConfigManager::getSystemConfig(initialized, sleepTime, deviceId, stationId);
        StaticJsonDocument<200> doc;
        JsonObject obj = doc.createNestedObject(NAMESPACE_SYSTEM);
        obj[KEY_INITIALIZED] = initialized;
        obj[KEY_SLEEP_TIME] = sleepTime;
        obj[KEY_DEVICE_ID] = deviceId;
        obj[KEY_STATION_ID] = stationId;

        String jsonString;
        serializeJson(doc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};

// Callback para NTC 100K usando JSON anidado en el namespace "ntc_100k"
class NTC100KConfigCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        // Se espera un JSON de la forma: { "ntc_100k": { <parámetros> } }
        StaticJsonDocument<200> fullDoc;
        DeserializationError error = deserializeJson(fullDoc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando NTC100K config: "));
            Serial.println(error.c_str());
            return;
        }
        JsonObject doc = fullDoc[NAMESPACE_NTC100K];
        ConfigManager::setNTC100KConfig(
            doc[KEY_NTC100K_T1] | 0.0,
            doc[KEY_NTC100K_R1] | 0.0,
            doc[KEY_NTC100K_T2] | 0.0,
            doc[KEY_NTC100K_R2] | 0.0,
            doc[KEY_NTC100K_T3] | 0.0,
            doc[KEY_NTC100K_R3] | 0.0
        );
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        double t1, r1, t2, r2, t3, r3;
        ConfigManager::getNTC100KConfig(t1, r1, t2, r2, t3, r3);
        
        StaticJsonDocument<200> fullDoc;
        // Crear objeto anidado con el namespace "ntc_100k"
        JsonObject doc = fullDoc.createNestedObject(NAMESPACE_NTC100K);
        doc[KEY_NTC100K_T1] = t1;
        doc[KEY_NTC100K_R1] = r1;
        doc[KEY_NTC100K_T2] = t2;
        doc[KEY_NTC100K_R2] = r2;
        doc[KEY_NTC100K_T3] = t3;
        doc[KEY_NTC100K_R3] = r3;
        
        String jsonString;
        serializeJson(fullDoc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};

// Callback para Conductividad usando JSON anidado en el namespace "cond"
class ConductivityConfigCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        // Se espera un JSON: { "cond": { <parámetros> } }
        StaticJsonDocument<200> fullDoc;
        DeserializationError error = deserializeJson(fullDoc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando Conductivity config: "));
            Serial.println(error.c_str());
            return;
        }
        JsonObject doc = fullDoc[NAMESPACE_COND];
        ConfigManager::setConductivityConfig(
            doc[KEY_CONDUCT_CT] | 0.0f,  // Temperatura de calibración
            doc[KEY_CONDUCT_CC] | 0.0f,  // Coeficiente de compensación
            doc[KEY_CONDUCT_V1] | 0.0f,
            doc[KEY_CONDUCT_T1] | 0.0f,
            doc[KEY_CONDUCT_V2] | 0.0f,
            doc[KEY_CONDUCT_T2] | 0.0f,
            doc[KEY_CONDUCT_V3] | 0.0f,
            doc[KEY_CONDUCT_T3] | 0.0f
        );
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        float calTemp, coefComp, v1, t1, v2, t2, v3, t3;
        ConfigManager::getConductivityConfig(calTemp, coefComp, v1, t1, v2, t2, v3, t3);
        
        StaticJsonDocument<200> fullDoc;
        JsonObject doc = fullDoc.createNestedObject(NAMESPACE_COND);
        doc[KEY_CONDUCT_CT] = calTemp;
        doc[KEY_CONDUCT_CC] = coefComp;
        doc[KEY_CONDUCT_V1] = v1;
        doc[KEY_CONDUCT_T1] = t1;
        doc[KEY_CONDUCT_V2] = v2;
        doc[KEY_CONDUCT_T2] = t2;
        doc[KEY_CONDUCT_V3] = v3;
        doc[KEY_CONDUCT_T3] = t3;
        
        String jsonString;
        serializeJson(fullDoc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};

// Callback para NTC 10K usando JSON anidado en el namespace "ntc_10k"
class NTC10KConfigCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        // Se espera JSON: { "ntc_10k": { <parámetros> } }
        StaticJsonDocument<200> fullDoc;
        DeserializationError error = deserializeJson(fullDoc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando NTC10K config: "));
            Serial.println(error.c_str());
            return;
        }
        JsonObject doc = fullDoc[NAMESPACE_NTC10K];
        ConfigManager::setNTC10KConfig(
            doc[KEY_NTC10K_T1] | 0.0,
            doc[KEY_NTC10K_R1] | 0.0,
            doc[KEY_NTC10K_T2] | 0.0,
            doc[KEY_NTC10K_R2] | 0.0,
            doc[KEY_NTC10K_T3] | 0.0,
            doc[KEY_NTC10K_R3] | 0.0
        );
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        double t1, r1, t2, r2, t3, r3;
        ConfigManager::getNTC10KConfig(t1, r1, t2, r2, t3, r3);
        
        StaticJsonDocument<200> fullDoc;
        JsonObject doc = fullDoc.createNestedObject(NAMESPACE_NTC10K);
        doc[KEY_NTC10K_T1] = t1;
        doc[KEY_NTC10K_R1] = r1;
        doc[KEY_NTC10K_T2] = t2;
        doc[KEY_NTC10K_R2] = r2;
        doc[KEY_NTC10K_T3] = t3;
        doc[KEY_NTC10K_R3] = r3;
        
        String jsonString;
        serializeJson(fullDoc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};

// Callback para pH usando JSON anidado en el namespace "ph"
class PHConfigCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        // Se espera JSON: { "ph": { <parámetros> } }
        StaticJsonDocument<200> fullDoc;
        DeserializationError error = deserializeJson(fullDoc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando pH config: "));
            Serial.println(error.c_str());
            return;
        }
        JsonObject doc = fullDoc[NAMESPACE_PH];
        ConfigManager::setPHConfig(
            doc[KEY_PH_V1] | 0.0f,
            doc[KEY_PH_T1] | 0.0f,
            doc[KEY_PH_V2] | 0.0f,
            doc[KEY_PH_T2] | 0.0f,
            doc[KEY_PH_V3] | 0.0f,
            doc[KEY_PH_T3] | 0.0f,
            doc[KEY_PH_CT] | 25.0f
        );
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        float v1, t1, v2, t2, v3, t3, calTemp;
        ConfigManager::getPHConfig(v1, t1, v2, t2, v3, t3, calTemp);
        
        StaticJsonDocument<200> fullDoc;
        JsonObject doc = fullDoc.createNestedObject(NAMESPACE_PH);
        doc[KEY_PH_V1] = v1;
        doc[KEY_PH_T1] = t1;
        doc[KEY_PH_V2] = v2;
        doc[KEY_PH_T2] = t2;
        doc[KEY_PH_V3] = v3;
        doc[KEY_PH_T3] = t3;
        doc[KEY_PH_CT] = calTemp;
        
        String jsonString;
        serializeJson(fullDoc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};

// Callback para Sensors (manteniendo su estructura original)
class SensorsConfigCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        // Se espera un JSON: { "sensors": [ {<sensor1>}, {<sensor2>}, ... ] }
        DynamicJsonDocument doc(2048);
        DeserializationError error = deserializeJson(doc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando Sensors config: "));
            Serial.println(error.c_str());
            return;
        }
        
        std::vector<SensorConfig> configs;
        JsonArray sensorArray = doc[NAMESPACE_SENSORS];
        
        for (JsonVariant sensor : sensorArray) {
            SensorConfig config;
            strncpy(config.configKey, sensor[KEY_SENSOR] | "", sizeof(config.configKey));
            strncpy(config.sensorId, sensor[KEY_SENSOR_ID] | "", sizeof(config.sensorId));
            strncpy(config.tempSensorId, sensor[KEY_SENSOR_TIMESTAMP] | "", sizeof(config.tempSensorId));
            config.type = static_cast<SensorType>(sensor[KEY_SENSOR_TYPE] | 0);
            config.enable = sensor[KEY_SENSOR_ENABLE] | false;
            

            configs.push_back(config);
        }
        
        ConfigManager::setSensorsConfigs(configs);
    }
    
    void onRead(BLECharacteristic *pCharacteristic) override {
        DynamicJsonDocument doc(2048);
        JsonArray sensorArray = doc.createNestedArray(NAMESPACE_SENSORS);

        std::vector<SensorConfig> configs = ConfigManager::getAllSensorConfigs();
        
        for (const auto& sensor : configs) {
            JsonObject obj = sensorArray.createNestedObject();
            obj[KEY_SENSOR]             = sensor.configKey;
            obj[KEY_SENSOR_ID]          = sensor.sensorId;
            obj[KEY_SENSOR_TYPE]        = static_cast<int>(sensor.type);
            obj[KEY_SENSOR_TIMESTAMP]   = sensor.tempSensorId;
            obj[KEY_SENSOR_ENABLE]      = sensor.enable;
        }

        String jsonString;
        serializeJson(doc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};

// Callback para LoRa usando JSON anidado en el namespace "lorawan"
class LoRaConfigCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        // Se espera JSON: { "lorawan": { <parámetros> } }
        StaticJsonDocument<256> fullDoc;
        DeserializationError error = deserializeJson(fullDoc, pCharacteristic->getValue());
        if (error) {
            Serial.print(F("Error deserializando LoRa config: "));
            Serial.println(error.c_str());
            return;
        }
        JsonObject doc = fullDoc[NAMESPACE_LORAWAN];
        uint32_t devAddr = doc[KEY_LORA_DEVADDR] | 0;
        String fNwkSIntKey = doc[KEY_LORA_FNWS_INTKEY] | "";
        String sNwkSIntKey = doc[KEY_LORA_SNWS_INTKEY] | "";
        String nwkSEncKey  = doc[KEY_LORA_NWKSENC_KEY]  | "";
        String appSKey     = doc[KEY_LORA_APPS_KEY]     | "";
        
        ConfigManager::setLoRaConfig(devAddr, fNwkSIntKey, sNwkSIntKey, nwkSEncKey, appSKey);
    }
    
    void onRead(BLECharacteristic* pCharacteristic) override {
        LoRaConfig config = ConfigManager::getLoRaConfig();
        
        StaticJsonDocument<256> fullDoc;
        JsonObject doc = fullDoc.createNestedObject(NAMESPACE_LORAWAN);
        doc[KEY_LORA_DEVADDR]     = config.devAddr;
        doc[KEY_LORA_FNWS_INTKEY] = config.fNwkSIntKey;
        doc[KEY_LORA_SNWS_INTKEY] = config.sNwkSIntKey;
        doc[KEY_LORA_NWKSENC_KEY]  = config.nwkSEncKey;
        doc[KEY_LORA_APPS_KEY]     = config.appSKey;
        
        String jsonString;
        serializeJson(fullDoc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
    }
};