#pragma once
#include <Preferences.h>
#include <vector>
#include <Arduino.h>  // Se incluye para utilizar el tipo String
#include "sensor_types.h"

// Definición de la estructura para la configuración de LoRa
struct LoRaConfig {
    uint32_t devAddr;
    String fNwkSIntKey;
    String sNwkSIntKey;
    String nwkSEncKey;
    String appSKey;
};

class ConfigManager {
public:
    // Banderas de inicialización
    static bool checkInitialized();
    static void initializeDefaultConfig();
    
    // Getters y setters para tiempo de sleep
    static uint32_t getSleepTime();
    static void setSleepTime(uint32_t time);
    
    // Getters y setters para el frame counter (fcnt)
    static uint32_t getFrameCounter();
    static void setFrameCounter(uint32_t fcnt);
    
    // Getters y setters para calibración NTC
    static void getNTC100KConfig(double& t1, double& r1, double& t2, double& r2, double& t3, double& r3);
    static void setNTC100KConfig(double t1, double r1, double t2, double r2, double t3, double r3);
    static void getNTC10KConfig(double& t1, double& r1, double& t2, double& r2, double& t3, double& r3);
    static void setNTC10KConfig(double t1, double r1, double t2, double r2, double t3, double r3);
    
    // Getters y setters para calibración de conductividad
    static void getConductivityConfig(float& calTemp, float& coefComp, 
                                    float& v1, float& t1, float& v2, float& t2, float& v3, float& t3);
    static void setConductivityConfig(float calTemp, float coefComp,
                                    float v1, float t1, float v2, float t2, float v3, float t3);
    
    // Getters y setters para calibración de pH
    static void getPHConfig(float& v1, float& t1, float& v2, float& t2, float& v3, float& t3, float& defaultTemp);
    static void setPHConfig(float v1, float t1, float v2, float t2, float v3, float t3, float defaultTemp);

    // Métodos para configuración de sensores
    static void setSensorsConfigs(const std::vector<SensorConfig>& configs);
    static std::vector<SensorConfig> getAllSensorConfigs();
    static void initializeSensorConfigs();

    // Métodos para la configuración de LoRa
    static LoRaConfig getLoRaConfig();
    static void setLoRaConfig(uint32_t devAddr, 
                              const String &fNwkSIntKey, 
                              const String &sNwkSIntKey, 
                              const String &nwkSEncKey, 
                              const String &appSKey);
    static String getDeviceId();
    static void setDeviceId(const String &id);


private:
    static const SensorConfig defaultConfigs[];
    static const char* INIT_KEY;
    static const bool INIT_VALUE;
    static const char* SENSOR_CONFIG_NAMESPACE;
    static const char* SYSTEM_NAMESPACE;
    static const char* SLEEP_NAMESPACE;
    static const char* NTC_NAMESPACE;
    static const char* COND_NAMESPACE;
    static const char* PH_NAMESPACE;
    static const char* LORAWAN_NAMESPACE;
    static const char* DEVICE_NAMESPACE;
}; 