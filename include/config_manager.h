#pragma once
#include <Preferences.h>
#include <vector>
#include "sensor_types.h"

class ConfigManager {
public:
    // Banderas de inicialización
    static bool checkInitialized();
    static void initializeDefaultConfig();
    
    // Getters y setters para tiempo de sleep
    static uint32_t getSleepTime();
    static void setSleepTime(uint32_t time);
    
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
    static void setSensorConfig(const char* sensorId, const char* sensorName, bool enable, const char* tempSensorId);
    static std::vector<SensorConfig> getAllSensorConfigs();
    static void initializeSensorConfigs();

private:
    static const char* INIT_KEY;
    static const bool INIT_VALUE;
    static const char* SENSOR_CONFIG_NAMESPACE;
}; 