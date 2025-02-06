#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>

enum SensorType {
    NTC_100K_TEMPERATURE_SENSOR,
    NTC_10K_TEMPERATURE_SENSOR,
    WATER_NTC_10K_TEMPERATURE_SENSOR,
    RTD_TEMPERATURE_SENSOR,
    DS18B20_TEMPERATURE_SENSOR, 
    PH_SENSOR,
    CONDUCTIVITY_SENSOR,
    CONDENSATION_HUMIDITY_SENSOR,
    SOIL_HUMIDITY_SENSOR,
};

struct SensorConfig {
    char configKey[20];
    char sensorId[20];
    SensorType type;
    uint8_t channel;
    char tempSensorId[20]; 
    bool enable;
};

struct SensorReading {
    char sensorId[20];
    SensorType type;
    float value;
};

#endif // SENSOR_TYPES_H