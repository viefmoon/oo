#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>

enum SensorType {
    N100K, // NTC 100K
    N10K, // NTC 10K
    WNTC10K, // Water NTC 10K
    RTD, // RTD
    DS18B20, // DS18B20
    PH, // PH
    COND, // Conductivity
    CONDH, // Condensation Humidity
    SOILH, // Soil Humidity
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