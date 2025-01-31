#include "config_manager.h"
#include <ArduinoJson.h>
#include <vector>
#include "config.h"
#include "sensor_types.h"

// Namespaces de configuración
const char* ConfigManager::INIT_KEY = "initialized";
const bool ConfigManager::INIT_VALUE = true;
const char* ConfigManager::SENSOR_CONFIG_NAMESPACE = "sensors";
const char* ConfigManager::SYSTEM_NAMESPACE = "system";
const char* ConfigManager::SLEEP_NAMESPACE = "sleep";
const char* ConfigManager::NTC_NAMESPACE = "ntc";
const char* ConfigManager::COND_NAMESPACE = "cond";
const char* ConfigManager::PH_NAMESPACE = "ph";
const char* ConfigManager::LORAWAN_NAMESPACE = "lorawan";
const char* ConfigManager::DEVICE_NAMESPACE = "device";

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
    Preferences prefs;
    prefs.begin(SYSTEM_NAMESPACE, true);
    bool isInit = prefs.getBool(INIT_KEY, false);
    prefs.end();
    return isInit;
}

void ConfigManager::initializeDefaultConfig() {
    Preferences prefs;
    
    // Sistema
    prefs.begin(SYSTEM_NAMESPACE, false);
    prefs.putBool(INIT_KEY, INIT_VALUE);
    prefs.end();
    
    // Sleep time en el namespace "sleep"
    prefs.begin(SLEEP_NAMESPACE, false);
    prefs.putUInt("sleep_time", DEFAULT_TIME_TO_SLEEP);
    prefs.end();
    
    // NTC 100K
    prefs.begin(NTC_NAMESPACE, false);
    prefs.putDouble("t1_100k", DEFAULT_T1_100K);
    prefs.putDouble("r1_100k", DEFAULT_R1_100K);
    prefs.putDouble("t2_100k", DEFAULT_T2_100K);
    prefs.putDouble("r2_100k", DEFAULT_R2_100K);
    prefs.putDouble("t3_100k", DEFAULT_T3_100K);
    prefs.putDouble("r3_100k", DEFAULT_R3_100K);
    
    // NTC 10K
    prefs.putDouble("t1_10k", DEFAULT_T1_10K);
    prefs.putDouble("r1_10k", DEFAULT_R1_10K);
    prefs.putDouble("t2_10k", DEFAULT_T2_10K);
    prefs.putDouble("r2_10k", DEFAULT_R2_10K);
    prefs.putDouble("t3_10k", DEFAULT_T3_10K);
    prefs.putDouble("r3_10k", DEFAULT_R3_10K);
    prefs.end();
    
    // Conductividad
    prefs.begin(COND_NAMESPACE, false);
    prefs.putFloat("cal_temp", CONDUCTIVITY_DEFAULT_TEMP);
    prefs.putFloat("coef", TEMP_COEF_COMPENSATION);
    prefs.putFloat("v1", CONDUCTIVITY_DEFAULT_V1);
    prefs.putFloat("t1", CONDUCTIVITY_DEFAULT_T1);
    prefs.putFloat("v2", CONDUCTIVITY_DEFAULT_V2);
    prefs.putFloat("t2", CONDUCTIVITY_DEFAULT_T2);
    prefs.putFloat("v3", CONDUCTIVITY_DEFAULT_V3);
    prefs.putFloat("t3", CONDUCTIVITY_DEFAULT_T3);
    prefs.end();
    
    // pH
    prefs.begin(PH_NAMESPACE, false);
    prefs.putFloat("v1", PH_DEFAULT_V1);
    prefs.putFloat("t1", PH_DEFAULT_T1);
    prefs.putFloat("v2", PH_DEFAULT_V2);
    prefs.putFloat("t2", PH_DEFAULT_T2);
    prefs.putFloat("v3", PH_DEFAULT_V3);
    prefs.putFloat("t3", PH_DEFAULT_T3);
    prefs.putFloat("cal_temp", PH_DEFAULT_TEMP);
    prefs.end();
    
    // Inicializar configuración de sensores
    initializeSensorConfigs();

    // Configuración de LoRa usando las constantes definidas al inicio, en su namespace "lorawan"
    prefs.begin(ConfigManager::LORAWAN_NAMESPACE, false);
    {
      StaticJsonDocument<256> doc;
      doc["devAddr"] = DEFAULT_LORA_DEVADDR;
      doc["fNwkSIntKey"] = DEFAULT_FNWKS_INTKEY;
      doc["sNwkSIntKey"] = DEFAULT_SNWKS_INTKEY;
      doc["nwkSEncKey"]  = DEFAULT_NWK_SENCKEY;
      doc["appSKey"]     = DEFAULT_APPS_KEY;
      String jsonString;
      serializeJson(doc, jsonString);
      prefs.putString("loraConfig", jsonString.c_str());
    }
    prefs.putUInt("fcnt", 0);
    prefs.end();
    
    // Agregar configuración del device
    prefs.begin(DEVICE_NAMESPACE, false);
    prefs.putString("device_id", DEFAULT_DEVICE_ID);
    prefs.end();
}

// =============== Sleep Time ===============
uint32_t ConfigManager::getSleepTime() {
    Preferences prefs;
    prefs.begin(SLEEP_NAMESPACE, true);
    uint32_t time = prefs.getUInt("sleep_time", DEFAULT_TIME_TO_SLEEP);
    prefs.end();
    return time;
}

void ConfigManager::setSleepTime(uint32_t time) {
    Preferences prefs;
    prefs.begin(SLEEP_NAMESPACE, false);
    prefs.putUInt("sleep_time", time);
    prefs.end();
}

// =============== Frame Counter (fcnt) ===============
uint32_t ConfigManager::getFrameCounter() {
    Preferences prefs;
    prefs.begin(LORAWAN_NAMESPACE, true);
    uint32_t fcnt = prefs.getUInt("fcnt", 0);
    prefs.end();
    return fcnt;
}

void ConfigManager::setFrameCounter(uint32_t fcnt) {
    Preferences prefs;
    prefs.begin(LORAWAN_NAMESPACE, false);
    prefs.putUInt("fcnt", fcnt);
    prefs.end();
}

// =============== NTC 100K Config ===============
void ConfigManager::getNTC100KConfig(double& t1, double& r1, double& t2, double& r2, double& t3, double& r3) {
    Preferences prefs;
    prefs.begin(NTC_NAMESPACE, true);
    
    t1 = prefs.getDouble("t1_100k", DEFAULT_T1_100K);
    r1 = prefs.getDouble("r1_100k", DEFAULT_R1_100K);
    t2 = prefs.getDouble("t2_100k", DEFAULT_T2_100K);
    r2 = prefs.getDouble("r2_100k", DEFAULT_R2_100K);
    t3 = prefs.getDouble("t3_100k", DEFAULT_T3_100K);
    r3 = prefs.getDouble("r3_100k", DEFAULT_R3_100K);
    
    prefs.end();
}

void ConfigManager::setNTC100KConfig(double t1, double r1, double t2, double r2, double t3, double r3) {
    Preferences prefs;
    prefs.begin(NTC_NAMESPACE, false);
    
    prefs.putDouble("t1_100k", t1);
    prefs.putDouble("r1_100k", r1);
    prefs.putDouble("t2_100k", t2);
    prefs.putDouble("r2_100k", r2);
    prefs.putDouble("t3_100k", t3);
    prefs.putDouble("r3_100k", r3);
    
    prefs.end();
}

// =============== NTC 10K Config ===============
void ConfigManager::getNTC10KConfig(double& t1, double& r1, double& t2, double& r2, double& t3, double& r3) {
    Preferences prefs;
    prefs.begin(NTC_NAMESPACE, true);
    
    t1 = prefs.getDouble("t1_10k", DEFAULT_T1_10K);
    r1 = prefs.getDouble("r1_10k", DEFAULT_R1_10K);
    t2 = prefs.getDouble("t2_10k", DEFAULT_T2_10K);
    r2 = prefs.getDouble("r2_10k", DEFAULT_R2_10K);
    t3 = prefs.getDouble("t3_10k", DEFAULT_T3_10K);
    r3 = prefs.getDouble("r3_10k", DEFAULT_R3_10K);
    
    prefs.end();
}

void ConfigManager::setNTC10KConfig(double t1, double r1, double t2, double r2, double t3, double r3) {
    Preferences prefs;
    prefs.begin(NTC_NAMESPACE, false);
    
    prefs.putDouble("t1_10k", t1);
    prefs.putDouble("r1_10k", r1);
    prefs.putDouble("t2_10k", t2);
    prefs.putDouble("r2_10k", r2);
    prefs.putDouble("t3_10k", t3);
    prefs.putDouble("r3_10k", r3);
    
    prefs.end();
}

// =============== Conductivity Config ===============
void ConfigManager::getConductivityConfig(float& calTemp, float& coefComp, 
                                        float& v1, float& t1, float& v2, float& t2, float& v3, float& t3) {
    Preferences prefs;
    prefs.begin(COND_NAMESPACE, true);
    
    calTemp = prefs.getFloat("cal_temp", CONDUCTIVITY_DEFAULT_TEMP);
    coefComp = prefs.getFloat("coef", TEMP_COEF_COMPENSATION);
    v1 = prefs.getFloat("v1", CONDUCTIVITY_DEFAULT_V1);
    t1 = prefs.getFloat("t1", CONDUCTIVITY_DEFAULT_T1);
    v2 = prefs.getFloat("v2", CONDUCTIVITY_DEFAULT_V2);
    t2 = prefs.getFloat("t2", CONDUCTIVITY_DEFAULT_T2);
    v3 = prefs.getFloat("v3", CONDUCTIVITY_DEFAULT_V3);
    t3 = prefs.getFloat("t3", CONDUCTIVITY_DEFAULT_T3);
    
    prefs.end();
}

void ConfigManager::setConductivityConfig(float calTemp, float coefComp,
                                        float v1, float t1, float v2, float t2, float v3, float t3) {
    Preferences prefs;
    prefs.begin(COND_NAMESPACE, false);
    
    prefs.putFloat("cal_temp", calTemp);
    prefs.putFloat("coef", coefComp);
    prefs.putFloat("v1", v1);
    prefs.putFloat("t1", t1);
    prefs.putFloat("v2", v2);
    prefs.putFloat("t2", t2);
    prefs.putFloat("v3", v3);
    prefs.putFloat("t3", t3);
    
    prefs.end();
}

// =============== pH Config ===============
void ConfigManager::getPHConfig(float& v1, float& t1, float& v2, float& t2, float& v3, float& t3, float& defaultTemp) {
    Preferences prefs;
    prefs.begin(PH_NAMESPACE, true);
    
    v1 = prefs.getFloat("v1", PH_DEFAULT_V1);
    t1 = prefs.getFloat("t1", PH_DEFAULT_T1);
    v2 = prefs.getFloat("v2", PH_DEFAULT_V2);
    t2 = prefs.getFloat("t2", PH_DEFAULT_T2);
    v3 = prefs.getFloat("v3", PH_DEFAULT_V3);
    t3 = prefs.getFloat("t3", PH_DEFAULT_T3);
    defaultTemp = prefs.getFloat("cal_temp", PH_DEFAULT_TEMP);
    
    prefs.end();
}

void ConfigManager::setPHConfig(float v1, float t1, float v2, float t2, float v3, float t3, float defaultTemp) {
    Preferences prefs;
    prefs.begin(PH_NAMESPACE, false);
    
    prefs.putFloat("v1", v1);
    prefs.putFloat("t1", t1);
    prefs.putFloat("v2", v2);
    prefs.putFloat("t2", t2);
    prefs.putFloat("v3", v3);
    prefs.putFloat("t3", t3);
    prefs.putFloat("cal_temp", defaultTemp);
    
    prefs.end();
}

std::vector<SensorConfig> ConfigManager::getAllSensorConfigs() {
    Preferences prefs;
    prefs.begin(SENSOR_CONFIG_NAMESPACE, true);
    std::vector<SensorConfig> configs;
    
    for(const auto& defaultConfig : ConfigManager::defaultConfigs) {
        String jsonString = prefs.getString(defaultConfig.configKey, "");
        
        if(jsonString.length() == 0) {
            continue;
        }
        
        StaticJsonDocument<128> doc;
        deserializeJson(doc, jsonString);
        
        SensorConfig config;
        strncpy(config.configKey, defaultConfig.configKey, sizeof(config.configKey));
        strncpy(config.sensorId, doc["id"] | "", sizeof(config.sensorId));
        config.type = static_cast<SensorType>(doc["t"] | 0);
        strncpy(config.tempSensorId, doc["ts"] | "", sizeof(config.tempSensorId));
        config.enable = doc["e"] | false;
        
        configs.push_back(config);
    }
    
    prefs.end();
    return configs;
}

void ConfigManager::initializeSensorConfigs() {
    Preferences prefs;
    prefs.begin(SENSOR_CONFIG_NAMESPACE, false);
    
    for (const auto& config : ConfigManager::defaultConfigs) {
        StaticJsonDocument<128> doc;
        
        doc["id"] = config.sensorId;
        doc["t"] = static_cast<int>(config.type);
        doc["ts"] = config.tempSensorId;
        doc["e"] = config.enable;
        
        String jsonString;
        serializeJson(doc, jsonString);
        prefs.putString(config.configKey, jsonString.c_str());
    }
    
    prefs.end();
}

void ConfigManager::setSensorsConfigs(const std::vector<SensorConfig>& configs) {
    Preferences prefs;
    prefs.begin(SENSOR_CONFIG_NAMESPACE, false);
    
    for (const auto& sensor : configs) {
        StaticJsonDocument<128> doc;
        doc["id"] = sensor.sensorId;
        doc["t"] = static_cast<int>(sensor.type);
        doc["ts"] = sensor.tempSensorId;
        doc["e"] = sensor.enable;
        
        String jsonString;
        serializeJson(doc, jsonString);
        prefs.putString(sensor.configKey, jsonString.c_str());
    }
    
    prefs.end();
}

// Obtener configuración de LoRa desde Preferences
LoRaConfig ConfigManager::getLoRaConfig() {
  Preferences prefs;
  prefs.begin(ConfigManager::LORAWAN_NAMESPACE, true);
  String jsonString = prefs.getString("loraConfig", "{}");
  prefs.end();
  
  StaticJsonDocument<256> doc;
  deserializeJson(doc, jsonString);
  
  LoRaConfig config;
  // Utilizamos los valores por defecto definidos al inicio del archivo
  config.devAddr = doc["devAddr"] | DEFAULT_LORA_DEVADDR;
  config.fNwkSIntKey = doc["fNwkSIntKey"] | DEFAULT_FNWKS_INTKEY;
  config.sNwkSIntKey = doc["sNwkSIntKey"] | DEFAULT_SNWKS_INTKEY;
  config.nwkSEncKey  = doc["nwkSEncKey"]  | DEFAULT_NWK_SENCKEY;
  config.appSKey     = doc["appSKey"]     | DEFAULT_APPS_KEY;
  
  return config;
}

// Actualizar configuración de LoRa en Preferences
void ConfigManager::setLoRaConfig(uint32_t devAddr, 
    const String &fNwkSIntKey, 
    const String &sNwkSIntKey, 
    const String &nwkSEncKey, 
    const String &appSKey) {
  Preferences prefs;
  prefs.begin(ConfigManager::LORAWAN_NAMESPACE, false);
  
  StaticJsonDocument<256> doc;
  doc["devAddr"] = devAddr;
  doc["fNwkSIntKey"] = fNwkSIntKey;
  doc["sNwkSIntKey"] = sNwkSIntKey;
  doc["nwkSEncKey"]  = nwkSEncKey;
  doc["appSKey"]     = appSKey;
  
  String jsonString;
  serializeJson(doc, jsonString);
  prefs.putString("loraConfig", jsonString.c_str());
  
  prefs.end();
}

String ConfigManager::getDeviceId() {
    Preferences prefs;
    prefs.begin(DEVICE_NAMESPACE, true);
    String id = prefs.getString("device_id", DEFAULT_DEVICE_ID);
    prefs.end();
    return id;

}

void ConfigManager::setDeviceId(const String &id) {
    Preferences prefs;
    prefs.begin(DEVICE_NAMESPACE, false);
    prefs.putString("device_id", id);
    prefs.end();

}
