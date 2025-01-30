#include "config_manager.h"
#include <ArduinoJson.h>
#include <vector>
#include "config_calib.h"
#include "config.h"
#include "sensor_types.h"

const char* ConfigManager::INIT_KEY = "initialized";
const bool ConfigManager::INIT_VALUE = true;
const char* ConfigManager::SENSOR_CONFIG_NAMESPACE = "sensors";

bool ConfigManager::checkInitialized() {
    Preferences prefs;
    prefs.begin("system", true);
    bool isInit = prefs.getBool(INIT_KEY, false);
    prefs.end();
    return isInit;
}

void ConfigManager::initializeDefaultConfig() {
    Preferences prefs;
    
    // Sistema
    prefs.begin("system", false);
    prefs.putBool(INIT_KEY, INIT_VALUE);
    prefs.end();
    
    // Sleep time
    prefs.begin("lorawan", false);
    prefs.putUInt("sleep_time", DEFAULT_TIME_TO_SLEEP);
    prefs.end();
    
    // NTC 100K
    prefs.begin("ntc", false);
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
    prefs.begin("cond", false);
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
    prefs.begin("ph", false);
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
}

// =============== Sleep Time ===============
uint32_t ConfigManager::getSleepTime() {
    Preferences prefs;
    prefs.begin("lorawan", true);
    uint32_t time = prefs.getUInt("sleep_time", DEFAULT_TIME_TO_SLEEP);
    prefs.end();
    return time;
}

void ConfigManager::setSleepTime(uint32_t time) {
    Preferences prefs;
    prefs.begin("lorawan", false);
    prefs.putUInt("sleep_time", time);
    prefs.end();
}

// =============== NTC 100K Config ===============
void ConfigManager::getNTC100KConfig(double& t1, double& r1, double& t2, double& r2, double& t3, double& r3) {
    Preferences prefs;
    prefs.begin("ntc", true);
    
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
    prefs.begin("ntc", false);
    
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
    prefs.begin("ntc", true);
    
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
    prefs.begin("ntc", false);
    
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
    prefs.begin("cond", true);
    
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
    prefs.begin("cond", false);
    
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
    prefs.begin("ph", true);
    
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
    prefs.begin("ph", false);
    
    prefs.putFloat("v1", v1);
    prefs.putFloat("t1", t1);
    prefs.putFloat("v2", v2);
    prefs.putFloat("t2", t2);
    prefs.putFloat("v3", v3);
    prefs.putFloat("t3", t3);
    prefs.putFloat("cal_temp", defaultTemp);
    
    prefs.end();
}

void ConfigManager::setSensorConfig(const char* sensorId, const char* sensorName, bool enable, const char* tempSensorId) {
    Preferences prefs;
    prefs.begin(SENSOR_CONFIG_NAMESPACE, false);
    
    String key = String(sensorId);
    
    // Guardar configuración como JSON string
    StaticJsonDocument<200> doc;
    doc["sensorName"] = sensorName;
    doc["enable"] = enable;
    doc["tempSensorId"] = tempSensorId;
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    prefs.putString(key.c_str(), jsonString);
    prefs.end();
}

std::vector<SensorConfig> ConfigManager::getAllSensorConfigs() {
    Preferences prefs;
    prefs.begin(SENSOR_CONFIG_NAMESPACE, true);
    
    std::vector<SensorConfig> configs;
    
    // Obtener la configuración predeterminada
    extern SensorConfig sensorConfigs[];
    extern const size_t NUM_SENSORS;
    
    for (size_t i = 0; i < NUM_SENSORS; i++) {
        SensorConfig config = sensorConfigs[i];
        
        // Buscar configuración guardada
        String jsonString = prefs.getString(config.sensorId, "");
        if (jsonString.length() > 0) {
            StaticJsonDocument<200> doc;
            deserializeJson(doc, jsonString);
            
            // Actualizar campos configurables
            strlcpy(config.sensorName, doc["sensorName"].as<const char*>(), sizeof(config.sensorName));
            config.enable = doc["enable"] | false;
            strlcpy(config.tempSensorId, doc["tempSensorId"].as<const char*>(), sizeof(config.tempSensorId));
        } else {
            // Si no hay configuración guardada, el sensor está deshabilitado por defecto
            config.enable = false;
            config.sensorName[0] = '\0';
            config.tempSensorId[0] = '\0';
        }
        
        configs.push_back(config);
    }
    
    prefs.end();
    return configs;
}

void ConfigManager::initializeSensorConfigs() {
    Preferences prefs;
    prefs.begin(SENSOR_CONFIG_NAMESPACE, false);
    prefs.clear();
    prefs.end();
}
