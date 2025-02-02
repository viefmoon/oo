#include "config_manager.h"
#include <ArduinoJson.h>
#include <vector>
#include "config.h"
#include "sensor_types.h"
#include <Preferences.h>
#include <Arduino.h> // Incluido para usar Serial

// Funciones auxiliares para leer y escribir el JSON completo en cada namespace.
static const size_t JSON_DOC_SIZE = 512;

static void writeNamespace(const char* ns, const StaticJsonDocument<JSON_DOC_SIZE>& doc) {
    Preferences prefs;
    prefs.begin(ns, false);
    String jsonString;
    serializeJson(doc, jsonString);
    // Se usa el mismo nombre del namespace como clave interna
    prefs.putString(ns, jsonString.c_str());
    prefs.end();
}

static void readNamespace(const char* ns, StaticJsonDocument<JSON_DOC_SIZE>& doc) {
    Preferences prefs;
    prefs.begin(ns, true);
    String jsonString = prefs.getString(ns, "{}");
    prefs.end();
    deserializeJson(doc, jsonString);
}

const SensorConfig ConfigManager::defaultConfigs[] = {
    {"0", "", NTC_100K_TEMPERATURE_SENSOR, 1, 0, "", false},
    {"1", "", NTC_100K_TEMPERATURE_SENSOR, 1, 1, "", false},
    {"2", "", CONDENSATION_HUMIDITY_SENSOR, 1, 2, "", false},
    {"3", "", SOIL_HUMIDITY_SENSOR, 1, 3, "", false},
    {"4", "", SOIL_HUMIDITY_SENSOR, 1, 4, "", false},
    {"5", "", CONDUCTIVITY_SENSOR, 1, 5, "", false},
    {"7", "", PH_SENSOR, 1, 7, "", false},
    {"R", "", RTD_TEMPERATURE_SENSOR, 0, 0, "", false},
    {"D", "", DS18B20_TEMPERATURE_SENSOR, 0, 0, "", false}
};

bool ConfigManager::checkInitialized() {
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_SYSTEM, doc);
    return doc[KEY_INITIALIZED] | false;
}

void ConfigManager::initializeDefaultConfig() {
    Serial.println("Inicializando configuración por defecto...");
    // Sistema unificado: NAMESPACE_SYSTEM (incluye system, sleep y device)
    {
        Serial.println("Configurando NAMESPACE_SYSTEM.");
        StaticJsonDocument<JSON_DOC_SIZE> doc;
        doc[KEY_STATION_ID] = DEFAULT_STATION_ID;
        doc[KEY_INITIALIZED] = VALUE_INITIALIZED;
        doc[KEY_SLEEP_TIME] = DEFAULT_TIME_TO_SLEEP;
        doc[KEY_DEVICE_ID] = DEFAULT_DEVICE_ID;
        writeNamespace(NAMESPACE_SYSTEM, doc);
        Serial.println("NAMESPACE_SYSTEM configurado.");
    }
    
    // NTC 100K: NAMESPACE_NTC100K
    {
        Serial.println("Configurando NAMESPACE_NTC100K.");
        StaticJsonDocument<JSON_DOC_SIZE> doc;
        doc[KEY_NTC100K_T1] = DEFAULT_T1_100K;
        doc[KEY_NTC100K_R1] = DEFAULT_R1_100K;
        doc[KEY_NTC100K_T2] = DEFAULT_T2_100K;
        doc[KEY_NTC100K_R2] = DEFAULT_R2_100K;
        doc[KEY_NTC100K_T3] = DEFAULT_T3_100K;
        doc[KEY_NTC100K_R3] = DEFAULT_R3_100K;
        writeNamespace(NAMESPACE_NTC100K, doc);
        Serial.println("NAMESPACE_NTC100K configurado.");
    }
    
    // NTC 10K: NAMESPACE_NTC10K
    {
        Serial.println("Configurando NAMESPACE_NTC10K.");
        StaticJsonDocument<JSON_DOC_SIZE> doc;
        doc[KEY_NTC10K_T1] = DEFAULT_T1_10K;
        doc[KEY_NTC10K_R1] = DEFAULT_R1_10K;
        doc[KEY_NTC10K_T2] = DEFAULT_T2_10K;
        doc[KEY_NTC10K_R2] = DEFAULT_R2_10K;
        doc[KEY_NTC10K_T3] = DEFAULT_T3_10K;
        doc[KEY_NTC10K_R3] = DEFAULT_R3_10K;
        writeNamespace(NAMESPACE_NTC10K, doc);
        Serial.println("NAMESPACE_NTC10K configurado.");
    }
    
    // Conductividad: NAMESPACE_COND
    {
        Serial.println("Configurando NAMESPACE_COND.");
        StaticJsonDocument<JSON_DOC_SIZE> doc;
        doc[KEY_CONDUCT_CT] = CONDUCTIVITY_DEFAULT_TEMP;
        doc[KEY_CONDUCT_CC] = TEMP_COEF_COMPENSATION;
        doc[KEY_CONDUCT_V1] = CONDUCTIVITY_DEFAULT_V1;
        doc[KEY_CONDUCT_T1] = CONDUCTIVITY_DEFAULT_T1;
        doc[KEY_CONDUCT_V2] = CONDUCTIVITY_DEFAULT_V2;
        doc[KEY_CONDUCT_T2] = CONDUCTIVITY_DEFAULT_T2;
        doc[KEY_CONDUCT_V3] = CONDUCTIVITY_DEFAULT_V3;
        doc[KEY_CONDUCT_T3] = CONDUCTIVITY_DEFAULT_T3;
        writeNamespace(NAMESPACE_COND, doc);
        Serial.println("NAMESPACE_COND configurado.");
    }
    
    // pH: NAMESPACE_PH
    {
        Serial.println("Configurando NAMESPACE_PH.");
        StaticJsonDocument<JSON_DOC_SIZE> doc;
        doc[KEY_PH_V1] = PH_DEFAULT_V1;
        doc[KEY_PH_T1] = PH_DEFAULT_T1;
        doc[KEY_PH_V2] = PH_DEFAULT_V2;
        doc[KEY_PH_T2] = PH_DEFAULT_T2;
        doc[KEY_PH_V3] = PH_DEFAULT_V3;
        doc[KEY_PH_T3] = PH_DEFAULT_T3;
        doc[KEY_PH_CT] = PH_DEFAULT_TEMP;
        writeNamespace(NAMESPACE_PH, doc);
        Serial.println("NAMESPACE_PH configurado.");
    }
    
    // Inicializar configuración de sensores
    initializeSensorConfigs();
    
    // LoRa: NAMESPACE_LORAWAN
    {
        Serial.println("Configurando NAMESPACE_LORAWAN.");
        StaticJsonDocument<JSON_DOC_SIZE> doc;
        doc[KEY_LORA_DEVADDR] = DEFAULT_LORA_DEVADDR;
        doc[KEY_LORA_FNWS_INTKEY] = DEFAULT_FNWKS_INTKEY;
        doc[KEY_LORA_SNWS_INTKEY] = DEFAULT_SNWKS_INTKEY;
        doc[KEY_LORA_NWKSENC_KEY]  = DEFAULT_NWK_SENCKEY;
        doc[KEY_LORA_APPS_KEY]     = DEFAULT_APPS_KEY;
        doc[KEY_FCNT] = 0;
        writeNamespace(NAMESPACE_LORAWAN, doc);
        Serial.println("NAMESPACE_LORAWAN configurado.");
    }
    
    Serial.println("Configuración por defecto inicializada.");
}

// Getter unificado para system, sleep y device:
void ConfigManager::getSystemConfig(bool &initialized, uint32_t &sleepTime, String &deviceId, String &stationId) {
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_SYSTEM, doc);
    initialized = doc[KEY_INITIALIZED] | false;
    sleepTime = doc[KEY_SLEEP_TIME] | DEFAULT_TIME_TO_SLEEP;
    deviceId = String(doc[KEY_DEVICE_ID] | DEFAULT_DEVICE_ID);
    stationId = String(doc[KEY_STATION_ID] | DEFAULT_STATION_ID);
}

// Setter unificado para system, sleep y device:
void ConfigManager::setSystemConfig(bool initialized, uint32_t sleepTime, const String &deviceId, const String &stationId) {
    Serial.print("Actualizando configuración del sistema: ");
    Serial.print("initialized=");
    Serial.print(initialized);
    Serial.print(", sleepTime=");
    Serial.print(sleepTime);
    Serial.print(", deviceId=");
    Serial.print(deviceId);
    Serial.print(", stationId=");
    Serial.println(stationId);
    
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_SYSTEM, doc);
    doc[KEY_INITIALIZED] = initialized;
    doc[KEY_SLEEP_TIME] = sleepTime;
    doc[KEY_DEVICE_ID] = deviceId;
    doc[KEY_STATION_ID] = stationId;
    writeNamespace(NAMESPACE_SYSTEM, doc);
    
    Serial.println("Configuración del sistema actualizada.");
}

// =============== Frame Counter (fcnt) ===============
uint32_t ConfigManager::getFrameCounter() {
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_LORAWAN, doc);
    return doc[KEY_FCNT] | 0;
}

void ConfigManager::setFrameCounter(uint32_t fcnt) {
    Serial.print("Actualizando frame counter: ");
    Serial.println(fcnt);
    
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_LORAWAN, doc);
    doc[KEY_FCNT] = fcnt;
    writeNamespace(NAMESPACE_LORAWAN, doc);
    
    Serial.println("Frame counter actualizado.");
}

// =============== NTC 100K Config ===============
void ConfigManager::getNTC100KConfig(double& t1, double& r1, double& t2, double& r2, double& t3, double& r3) {
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_NTC100K, doc);
    t1 = doc[KEY_NTC100K_T1] | DEFAULT_T1_100K;
    r1 = doc[KEY_NTC100K_R1] | DEFAULT_R1_100K;
    t2 = doc[KEY_NTC100K_T2] | DEFAULT_T2_100K;
    r2 = doc[KEY_NTC100K_R2] | DEFAULT_R2_100K;
    t3 = doc[KEY_NTC100K_T3] | DEFAULT_T3_100K;
    r3 = doc[KEY_NTC100K_R3] | DEFAULT_R3_100K;
}

void ConfigManager::setNTC100KConfig(double t1, double r1, double t2, double r2, double t3, double r3) {
    Serial.print("Actualizando configuración NTC 100K: t1=");
    Serial.print(t1);
    Serial.print(", r1=");
    Serial.print(r1);
    Serial.print(", t2=");
    Serial.print(t2);
    Serial.print(", r2=");
    Serial.print(r2);
    Serial.print(", t3=");
    Serial.print(t3);
    Serial.print(", r3=");
    Serial.println(r3);
    
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_NTC100K, doc);
    doc[KEY_NTC100K_T1] = t1;
    doc[KEY_NTC100K_R1] = r1;
    doc[KEY_NTC100K_T2] = t2;
    doc[KEY_NTC100K_R2] = r2;
    doc[KEY_NTC100K_T3] = t3;
    doc[KEY_NTC100K_R3] = r3;
    writeNamespace(NAMESPACE_NTC100K, doc);
    
    Serial.println("Configuración NTC 100K actualizada.");
}

// =============== NTC 10K Config ===============
void ConfigManager::getNTC10KConfig(double& t1, double& r1, double& t2, double& r2, double& t3, double& r3) {
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_NTC10K, doc);
    t1 = doc[KEY_NTC10K_T1] | DEFAULT_T1_10K;
    r1 = doc[KEY_NTC10K_R1] | DEFAULT_R1_10K;
    t2 = doc[KEY_NTC10K_T2] | DEFAULT_T2_10K;
    r2 = doc[KEY_NTC10K_R2] | DEFAULT_R2_10K;
    t3 = doc[KEY_NTC10K_T3] | DEFAULT_T3_10K;
    r3 = doc[KEY_NTC10K_R3] | DEFAULT_R3_10K;
}

void ConfigManager::setNTC10KConfig(double t1, double r1, double t2, double r2, double t3, double r3) {
    Serial.print("Actualizando configuración NTC 10K: t1=");
    Serial.print(t1);
    Serial.print(", r1=");
    Serial.print(r1);
    Serial.print(", t2=");
    Serial.print(t2);
    Serial.print(", r2=");
    Serial.print(r2);
    Serial.print(", t3=");
    Serial.print(t3);
    Serial.print(", r3=");
    Serial.println(r3);
    
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_NTC10K, doc);
    doc[KEY_NTC10K_T1] = t1;
    doc[KEY_NTC10K_R1] = r1;
    doc[KEY_NTC10K_T2] = t2;
    doc[KEY_NTC10K_R2] = r2;
    doc[KEY_NTC10K_T3] = t3;
    doc[KEY_NTC10K_R3] = r3;
    writeNamespace(NAMESPACE_NTC10K, doc);
    
    Serial.println("Configuración NTC 10K actualizada.");
}

// =============== Conductivity Config ===============
void ConfigManager::getConductivityConfig(float& calTemp, float& coefComp, 
                                        float& v1, float& t1, float& v2, float& t2, float& v3, float& t3) {
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_COND, doc);
    calTemp = doc[KEY_CONDUCT_CT] | CONDUCTIVITY_DEFAULT_TEMP;
    coefComp = doc[KEY_CONDUCT_CC] | TEMP_COEF_COMPENSATION;
    v1 = doc[KEY_CONDUCT_V1] | CONDUCTIVITY_DEFAULT_V1;
    t1 = doc[KEY_CONDUCT_T1] | CONDUCTIVITY_DEFAULT_T1;
    v2 = doc[KEY_CONDUCT_V2] | CONDUCTIVITY_DEFAULT_V2;
    t2 = doc[KEY_CONDUCT_T2] | CONDUCTIVITY_DEFAULT_T2;
    v3 = doc[KEY_CONDUCT_V3] | CONDUCTIVITY_DEFAULT_V3;
    t3 = doc[KEY_CONDUCT_T3] | CONDUCTIVITY_DEFAULT_T3;
}

void ConfigManager::setConductivityConfig(float calTemp, float coefComp,
                                        float v1, float t1, float v2, float t2, float v3, float t3) {
    Serial.print("Actualizando configuración de Conductividad: calTemp=");
    Serial.print(calTemp);
    Serial.print(", coefComp=");
    Serial.print(coefComp);
    Serial.print(", v1=");
    Serial.print(v1);
    Serial.print(", t1=");
    Serial.print(t1);
    Serial.print(", v2=");
    Serial.print(v2);
    Serial.print(", t2=");
    Serial.print(t2);
    Serial.print(", v3=");
    Serial.print(v3);
    Serial.print(", t3=");
    Serial.println(t3);
    
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_COND, doc);
    doc[KEY_CONDUCT_CT] = calTemp;
    doc[KEY_CONDUCT_CC] = coefComp;
    doc[KEY_CONDUCT_V1] = v1;
    doc[KEY_CONDUCT_T1] = t1;
    doc[KEY_CONDUCT_V2] = v2;
    doc[KEY_CONDUCT_T2] = t2;
    doc[KEY_CONDUCT_V3] = v3;
    doc[KEY_CONDUCT_T3] = t3;
    writeNamespace(NAMESPACE_COND, doc);
    
    Serial.println("Configuración de Conductividad actualizada.");
}

// =============== pH Config ===============
void ConfigManager::getPHConfig(float& v1, float& t1, float& v2, float& t2, float& v3, float& t3, float& defaultTemp) {
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_PH, doc);
    v1 = doc[KEY_PH_V1] | PH_DEFAULT_V1;
    t1 = doc[KEY_PH_T1] | PH_DEFAULT_T1;
    v2 = doc[KEY_PH_V2] | PH_DEFAULT_V2;
    t2 = doc[KEY_PH_T2] | PH_DEFAULT_T2;
    v3 = doc[KEY_PH_V3] | PH_DEFAULT_V3;
    t3 = doc[KEY_PH_T3] | PH_DEFAULT_T3;
    defaultTemp = doc[KEY_PH_CT] | PH_DEFAULT_TEMP;
}

void ConfigManager::setPHConfig(float v1, float t1, float v2, float t2, float v3, float t3, float defaultTemp) {
    Serial.print("Actualizando configuración de pH: v1=");
    Serial.print(v1);
    Serial.print(", t1=");
    Serial.print(t1);
    Serial.print(", v2=");
    Serial.print(v2);
    Serial.print(", t2=");
    Serial.print(t2);
    Serial.print(", v3=");
    Serial.print(v3);
    Serial.print(", t3=");
    Serial.print(t3);
    Serial.print(", defaultTemp=");
    Serial.println(defaultTemp);
    
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_PH, doc);
    doc[KEY_PH_V1] = v1;
    doc[KEY_PH_T1] = t1;
    doc[KEY_PH_V2] = v2;
    doc[KEY_PH_T2] = t2;
    doc[KEY_PH_V3] = v3;
    doc[KEY_PH_T3] = t3;
    doc[KEY_PH_CT] = defaultTemp;
    writeNamespace(NAMESPACE_PH, doc);
    
    Serial.println("Configuración de pH actualizada.");
}

std::vector<SensorConfig> ConfigManager::getAllSensorConfigs() {
    std::vector<SensorConfig> configs;
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_SENSORS, doc);
    
    if (!doc.is<JsonArray>()) {
        // Si no es un arreglo, no hay nada que leer
        return configs;
    }
    
    JsonArray sensorArray = doc.as<JsonArray>();
    for (JsonObject sensorObj : sensorArray) {
        SensorConfig config;
        const char* cKey = sensorObj[KEY_SENSOR] | "";
        strncpy(config.configKey, cKey, sizeof(config.configKey));
        const char* sensorId = sensorObj[KEY_SENSOR_ID] | "";
        strncpy(config.sensorId, sensorId, sizeof(config.sensorId));
        config.type = static_cast<SensorType>(sensorObj[KEY_SENSOR_TYPE] | 0);
        const char* tempSensorId = sensorObj[KEY_SENSOR_TIMESTAMP] | "";
        strncpy(config.tempSensorId, tempSensorId, sizeof(config.tempSensorId));
        config.enable = sensorObj[KEY_SENSOR_ENABLE] | false;
        
        configs.push_back(config);
    }
    
    return configs;
}

void ConfigManager::initializeSensorConfigs() {
    Serial.println("Inicializando configuraciones por defecto de sensores.");
    Preferences prefs;
    prefs.begin(NAMESPACE_SENSORS, false);
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    JsonArray sensorArray = doc.to<JsonArray>(); // Array raíz

    size_t defaultCount = sizeof(ConfigManager::defaultConfigs) / sizeof(SensorConfig);
    Serial.print("Se inicializarán ");
    Serial.print(defaultCount);
    Serial.println(" configuraciones de sensores.");

    for (const auto& config : ConfigManager::defaultConfigs) {
        JsonObject sensorObj = sensorArray.createNestedObject();
        sensorObj[KEY_SENSOR] = config.configKey;
        sensorObj[KEY_SENSOR_ID] = config.sensorId;
        sensorObj[KEY_SENSOR_TYPE] = static_cast<int>(config.type);
        sensorObj[KEY_SENSOR_TIMESTAMP] = config.tempSensorId;
        sensorObj[KEY_SENSOR_ENABLE] = config.enable;
    }
    
    String jsonString;
    serializeJson(doc, jsonString);
    // Se guarda usando el mismo nombre del namespace
    prefs.putString(NAMESPACE_SENSORS, jsonString.c_str());
    prefs.end();
    Serial.println("Configuración de sensores por defecto inicializada.");
}

void ConfigManager::setSensorsConfigs(const std::vector<SensorConfig>& configs) {
    Serial.print("Actualizando configuraciones de sensores, cantidad: ");
    Serial.println(configs.size());
    Preferences prefs;
    prefs.begin(NAMESPACE_SENSORS, false);
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    JsonArray sensorArray = doc.to<JsonArray>();
    
    for (const auto& sensor : configs) {
        JsonObject sensorObj = sensorArray.createNestedObject();
        sensorObj[KEY_SENSOR] = sensor.configKey;
        sensorObj[KEY_SENSOR_ID] = sensor.sensorId;
        sensorObj[KEY_SENSOR_TYPE] = static_cast<int>(sensor.type);
        sensorObj[KEY_SENSOR_TIMESTAMP] = sensor.tempSensorId;
        sensorObj[KEY_SENSOR_ENABLE] = sensor.enable;
    }
    
    String jsonString;
    serializeJson(doc, jsonString);
    prefs.putString(NAMESPACE_SENSORS, jsonString.c_str());
    prefs.end();
    Serial.println("Configuraciones de sensores actualizadas.");
}

// Obtener configuración de LoRa desde Preferences
LoRaConfig ConfigManager::getLoRaConfig() {
    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_LORAWAN, doc);
    
    LoRaConfig config;
    config.devAddr    = doc[KEY_LORA_DEVADDR] | DEFAULT_LORA_DEVADDR;
    config.fNwkSIntKey = doc[KEY_LORA_FNWS_INTKEY] | DEFAULT_FNWKS_INTKEY;
    config.sNwkSIntKey = doc[KEY_LORA_SNWS_INTKEY] | DEFAULT_SNWKS_INTKEY;
    config.nwkSEncKey  = doc[KEY_LORA_NWKSENC_KEY]  | DEFAULT_NWK_SENCKEY;
    config.appSKey     = doc[KEY_LORA_APPS_KEY]     | DEFAULT_APPS_KEY;
    
    return config;
}

// Actualizar configuración de LoRa en Preferences
void ConfigManager::setLoRaConfig(uint32_t devAddr, 
    const String &fNwkSIntKey, 
    const String &sNwkSIntKey, 
    const String &nwkSEncKey, 
    const String &appSKey) {
    Serial.print("Actualizando configuración LoRa: devAddr=");
    Serial.print(devAddr);
    Serial.print(", fNwkSIntKey=");
    Serial.print(fNwkSIntKey);
    Serial.print(", sNwkSIntKey=");
    Serial.print(sNwkSIntKey);
    Serial.print(", nwkSEncKey=");
    Serial.print(nwkSEncKey);
    Serial.print(", appSKey=");
    Serial.println(appSKey);

    StaticJsonDocument<JSON_DOC_SIZE> doc;
    readNamespace(NAMESPACE_LORAWAN, doc);
    doc[KEY_LORA_DEVADDR] = devAddr;
    doc[KEY_LORA_FNWS_INTKEY] = fNwkSIntKey;
    doc[KEY_LORA_SNWS_INTKEY] = sNwkSIntKey;
    doc[KEY_LORA_NWKSENC_KEY]  = nwkSEncKey;
    doc[KEY_LORA_APPS_KEY]     = appSKey;
    writeNamespace(NAMESPACE_LORAWAN, doc);
    
    Serial.println("Configuración LoRa actualizada.");
}
