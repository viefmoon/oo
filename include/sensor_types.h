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
    VOLTAGE_SENSOR

};

struct SensorConfig {
    char sensorId[20];
    char sensorName[20];
    SensorType type;
    uint8_t adcNumber;
    uint8_t channel;
    char tempSensorId[20]; 
    char enable[20];
};

struct SensorReading {
    char sensorId[20];
    char sensorName[20];
    SensorType type;
    float value;
    uint32_t timestamp;
};

#endif // SENSOR_TYPES_H