#include "SensorManager.h"
#include <Wire.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include "ADS131M08.h"
#include "MAX31865.h"
#include "RTCManager.h"
#include "sensor_types.h"
#include "config.h"
#include <Preferences.h>
#include "config_calib.h"
#include "config_manager.h"



// =============== Implementaciones de los métodos de SensorManager ===============

void SensorManager::beginSensors() {
    // Inicializar pines de SPI (CS) y luego SPI
    initializeSPIPins();
    spi.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

    // Encender alimentación 3.3V (si lo maneja el PowerManager)
    powerManager.power3V3On();

    // Inicializar RTD (MAX31865)
    rtd.begin();
    {
        bool vBias = true;
        bool autoConvert = true;
        bool oneShot = false;
        bool threeWire = false;
        uint8_t faultCycle = MAX31865_FAULT_DETECTION_NONE;
        bool faultClear = true;
        bool filter50Hz = true;
        uint16_t lowTh = 0x0000;
        uint16_t highTh = 0x7fff;
        rtd.configure(vBias, autoConvert, oneShot, threeWire, faultCycle, faultClear, filter50Hz, lowTh, highTh);
    }

    // Inicializar ADC 08
    adc.begin(ADC_CS_PIN, ADC_DRDY_PIN, ADC_RST_PIN);
    adc.hardReset();
    delay(10);
    adc.setPowerMode(POWER_MODE_HIGH_RESOLUTION);
    adc.setOsr(OSR_16384);

    // Inicializar DS18B20
    dallasTemp.begin();
    dallasTemp.setResolution(12);
    // Lectura inicial (descartable)
    dallasTemp.requestTemperatures();
    delay(750);
    dallasTemp.getTempCByIndex(0);
}

void SensorManager::initializeSPIPins() {
    // Inicializar SS del LORA que está conectado directamente al ESP32
    pinMode(LORA_NSS_PIN, OUTPUT);
    digitalWrite(LORA_NSS_PIN, HIGH);

    // Inicializar SS conectados al expansor I2C
    ioExpander.pinMode(P03, OUTPUT);  
    ioExpander.pinMode(P05, OUTPUT);  

    // Establecer todos los SS del expansor en HIGH
    ioExpander.digitalWrite(P03, HIGH);
    ioExpander.digitalWrite(P05, HIGH);
}

// ========== Funciones de conversión ==========
float convertNTC100K(float voltage) {
    // Obtener valores de calibración usando ConfigManager
    double T1, R1, T2, R2, T3, R3;
    ConfigManager::getNTC100KConfig(T1, R1, T2, R2, T3, R3);
    
    // Convertir temperaturas a Kelvin para cálculos
    const double T1K = T1 + 273.15;
    const double T2K = T2 + 273.15;
    const double T3K = T3 + 273.15;
    
    // El voltaje medido es (V_rama_izq - V_rama_der)
    // Cuando NTC = 100k: V_diff = 0 → R_ntc = 100k
    double V_diff = voltage; // Ya viene como diferencial
    
    // Fórmula corregida para puente balanceado:
    double R_ntc = R1 * (3.3/2.0 - V_diff) / (3.3/2.0 + V_diff);
    
    // Cálculo de coeficientes Steinhart-Hart
    const double L1 = log(R1);
    const double L2 = log(R2);
    const double L3 = log(R3);
    
    const double Y1 = 1.0/T1K;
    const double Y2 = 1.0/T2K;
    const double Y3 = 1.0/T3K;
    
    const double gamma2 = (Y2 - Y1)/(L2 - L1);
    const double gamma3 = (Y3 - Y1)/(L3 - L1);
    
    const double C = (gamma3 - gamma2)/(L3 - L2) * (1.0/(L1 + L2 + L3));
    const double B = gamma2 - C*(pow(L1,2) + L1*L2 + pow(L2,2));
    const double A = Y1 - L1*(B + C*pow(L1,2));
    
    // Aplicar ecuación de Steinhart-Hart completa
    const double lnR = log(R_ntc);
    const double invT = A + B*lnR + C*pow(lnR,3);
    const double temperature = (1.0/invT) - 273.15;  // Convertir a °C
    
    return (float)temperature;
}

float convertNTC10K(float voltage) {
    // Obtener valores de calibración usando ConfigManager
    double T1, R1, T2, R2, T3, R3;
    ConfigManager::getNTC10KConfig(T1, R1, T2, R2, T3, R3);
    
    // Convertir temperaturas a Kelvin para cálculos
    const double T1K = T1 + 273.15;
    const double T2K = T2 + 273.15;
    const double T3K = T3 + 273.15;
    
    // El voltaje medido es (V_rama_izq - V_rama_der)
    // Cuando NTC = 10k: V_diff = 0 → R_ntc = 10k
    double V_diff = voltage; // Ya viene como diferencial
    
    // Fórmula corregida para puente balanceado:
    double R_ntc = R1 * (3.3/2.0 - V_diff) / (3.3/2.0 + V_diff);
    
    // Cálculo de coeficientes Steinhart-Hart
    const double L1 = log(R1);
    const double L2 = log(R2);
    const double L3 = log(R3);
    
    const double Y1 = 1.0/T1K;
    const double Y2 = 1.0/T2K;
    const double Y3 = 1.0/T3K;
    
    const double gamma2 = (Y2 - Y1)/(L2 - L1);
    const double gamma3 = (Y3 - Y1)/(L3 - L1);
    
    const double C = (gamma3 - gamma2)/(L3 - L2) * (1.0/(L1 + L2 + L3));
    const double B = gamma2 - C*(pow(L1,2) + L1*L2 + pow(L2,2));
    const double A = Y1 - L1*(B + C*pow(L1,2));
    
    // Aplicar ecuación de Steinhart-Hart completa
    const double lnR = log(R_ntc);
    const double invT = A + B*lnR + C*pow(lnR,3);
    const double temperature = (1.0/invT) - 273.15;  // Convertir a °C
    
    return (float)temperature;
}

float convertHDS10(float voltage) {
    // El voltaje recibido es el diferencial (V_izq - V_der)
    const float V_diff = voltage; // Ya viene como diferencial
                                    // se calculo experimentalmente

    // Ecuación cuadrática obtenida de la regresión (H = aV² + bV + c)
    const float a = -147.445f;  // Coeficiente cuadrático
    const float b = 13.088f;    // Coeficiente lineal 
    const float c = 100.302f;   // Término independiente
    
    float humidity = a * pow(V_diff, 2) + b * V_diff + c;
    
    // Limitar valores a rango físico posible (0-100%)
    humidity = constrain(humidity, 0.0f, 100.0f);
    
    return humidity;
}

float convertSoilMoisture(float voltage) {
    // Conversión lineal: 0V = 0%, 1.17V = 100%
    const float SOIL_MOISTURE_FULL_SCALE_V = 1.17f;  // Voltaje al 100% de humedad
    const float slope = 100.0f / SOIL_MOISTURE_FULL_SCALE_V;
    float humidity = voltage * slope;
    
    // Limitar valores a rango físico posible (0-100%)
    humidity = constrain(humidity, 0.0f, 100.0f);
    
    return humidity;
}

float convertConductivity(float voltage, float solutionTemp) {
    // Obtener valores de calibración usando ConfigManager
    float calTemp, coefComp, V1, T1, V2, T2, V3, T3;
    ConfigManager::getConductivityConfig(calTemp, coefComp, V1, T1, V2, T2, V3, T3);

    // Si solutionTemp es NAN, usar la temperatura de calibración como valor por defecto
    if (isnan(solutionTemp)) {
        solutionTemp = calTemp;
        Serial.println("Advertencia: Usando temperatura de calibración por defecto para conductividad");
    }

    // Matriz para resolver el sistema de ecuaciones
    // Basado en 3 puntos de calibración
    const double det = V1*V1*(V2 - V3) - V1*(V2*V2 - V3*V3) + (V2*V2*V3 - V2*V3*V3);
    
    // Calcular coeficientes solo si el determinante no es cero
    if(fabs(det) > 1e-6) {
        const double a = (T1*(V2 - V3) - T2*(V1 - V3) + T3*(V1 - V2)) / det;
        const double b = (T1*(V3*V3 - V2*V2) + T2*(V1*V1 - V3*V3) + T3*(V2*V2 - V1*V1)) / det;
        const double c = (T1*(V2*V2*V3 - V2*V3*V3) - T2*(V1*V1*V3 - V1*V3*V3) + T3*(V1*V1*V2 - V1*V2*V2)) / det;
        
        // Aplicar compensación de temperatura usando el parámetro recibido
        const double compensation = 1.0 + coefComp * (solutionTemp - calTemp);
        double compensatedVoltage = voltage / compensation;
        double conduc = a * (compensatedVoltage * compensatedVoltage) 
                    + b * compensatedVoltage 
                    + c;

        // Asegura que no sea negativo
        return fmax(conduc, 0.0);
    }
    else {
        return NAN;
    }
}


float convertBattery(float voltage) {
    const double R1 = 470000.0;  // Resistencia R1 del divisor
    const double R2 = 1500000.0; // Resistencia R2 del divisor
    const double conversionFactor = (R1 + R2) / R1;
    return (float)(voltage * conversionFactor);
}

float convertPH(float voltage, float solutionTemp) {
    // Obtener valores de calibración usando ConfigManager
    float V1, T1, V2, T2, V3, T3, TEMP_CAL;
    ConfigManager::getPHConfig(V1, T1, V2, T2, V3, T3, TEMP_CAL);

    // Si solutionTemp es NAN, usar la temperatura de calibración como valor por defecto
    if (isnan(solutionTemp)) {
        solutionTemp = TEMP_CAL;
        Serial.println("Advertencia: Usando temperatura de calibración por defecto para pH");
    }

    // Imprimir estado de calibración
    Serial.println("Estado de calibración pH:");
    Serial.print("TEMP_CAL: "); 
    Serial.println(TEMP_CAL);
    Serial.println("Punto 1:");
    Serial.print("V1: "); 
    Serial.println(V1);
    Serial.print("T1: "); 
    Serial.println(T1);
    Serial.println("Punto 2:");
    Serial.print("V2: "); 
    Serial.println(V2);
    Serial.print("T2: "); 
    Serial.println(T2);
    Serial.println("Punto 3:");
    Serial.print("V3: "); 
    Serial.println(V3);
    Serial.print("T3: "); 
    Serial.println(T3);

    // Datos de calibración (pH, voltaje)
    const double pH_calib[] = {T1, T2, T3};
    const double V_calib[] = {V1, V2, V3};
    const int n = 3; // Número de puntos de calibración

    // Calcular sumatorias necesarias para mínimos cuadrados
    double sum_pH = 0.0;
    double sum_V = 0.0;
    double sum_pHV = 0.0;
    double sum_pH2 = 0.0;

    for (int i = 0; i < n; i++) {
        sum_pH += pH_calib[i];
        sum_V += V_calib[i];
        sum_pHV += pH_calib[i] * V_calib[i];
        sum_pH2 += pH_calib[i] * pH_calib[i];
    }

    // Calcular la pendiente S usando mínimos cuadrados
    double S_CAL = ((n * sum_pHV) - (sum_pH * sum_V)) / ((n * sum_pH2) - (sum_pH * sum_pH));

    // Calcular el offset E0 usando mínimos cuadrados
    double E0 = ((sum_V) + (S_CAL * sum_pH)) / n;

    // Ajustar la pendiente según la temperatura actual usando la ecuación de Nernst
    const double tempK = (solutionTemp + 273.15);
    const double tempCalK = (TEMP_CAL + 273.15);
    const double S_T = S_CAL * (tempK / tempCalK);

    // Calcular pH usando la ecuación de Nernst ajustada: pH = (E0 - E) / S(T)
    double pH = ((E0 + voltage) / S_T);
    // Limitar el pH a un rango físicamente posible (0-14)
    pH = constrain(pH, 0.0, 14.0);

    return pH;
}

// Añadir variable estática para almacenar las lecturas del ADC
static AdcOutput lastAdcReading = {0};
static bool adcReadingValid = false;

bool SensorManager::updateADCReadings(uint32_t timeout_ms) {
    unsigned long startTime = millis();
    adcReadingValid = false;
    
    // Esperar hasta que haya datos disponibles o se alcance el timeout
    while ((millis() - startTime) < timeout_ms) {
        if (adc.isDataReady()) {
            lastAdcReading = adc.readAdcFloat();
            adcReadingValid = true;
            
            // Imprimir valores crudos para debug
            Serial.println("Lectura ADC completa:");
            for(int i = 0; i < 8; i++) {
                Serial.print("Canal ");
                Serial.print(i);
                Serial.print(": ");
                Serial.print(lastAdcReading.ch[i].f, 8);
                Serial.println(" V");
            }
            
            return true;
        }
        delay(1); // Pequeña pausa para no saturar el CPU
    }
    
    // Si llegamos aquí, hubo timeout
    Serial.println("Timeout en lectura ADC - usando valores 0V");
    for(int i = 0; i < 8; i++) {
        lastAdcReading.ch[i].f = 0.0f;
    }
    return false;
}

float SensorManager::readAnalogSensor(const SensorConfig &cfg) {
    // Usar el valor almacenado del último updateADCReadings()
    float rawValue = lastAdcReading.ch[cfg.channel].f;
    
    // Aplicar conversión según el tipo de sensor
    switch(cfg.type) {
        case NTC_100K_TEMPERATURE_SENSOR:
            return convertNTC100K(rawValue);
        case NTC_10K_TEMPERATURE_SENSOR:
            return convertNTC10K(rawValue);
        case PH_SENSOR: {
            float solutionTemp = NAN;
            if(strlen(cfg.tempSensorId) > 0) {
                // Buscar el sensor de temperatura asociado
                for(size_t i = 0; i < NUM_SENSORS; i++) {
                    if(strcmp(sensorConfigs[i].sensorId, cfg.tempSensorId) == 0) {
                        solutionTemp = getSensorReading(sensorConfigs[i]).value;
                        break;
                    }
                }
            }
            return convertPH(rawValue, solutionTemp);
        }
        case CONDUCTIVITY_SENSOR: {
            float solutionTemp = NAN;
            if(strlen(cfg.tempSensorId) > 0) {
                // Buscar el sensor de temperatura asociado
                for(size_t i = 0; i < NUM_SENSORS; i++) {
                    if(strcmp(sensorConfigs[i].sensorId, cfg.tempSensorId) == 0) {
                        solutionTemp = getSensorReading(sensorConfigs[i]).value;
                        break;
                    }
                }
            }
            return convertConductivity(rawValue, solutionTemp);
        }
        case SOIL_HUMIDITY_SENSOR:
            return convertSoilMoisture(rawValue);

        case CONDENSATION_HUMIDITY_SENSOR: 
            return convertHDS10(rawValue);
        case VOLTAGE_SENSOR:
            return convertBattery(rawValue);
        default:
            Serial.print("Tipo de sensor no manejado: ");
            Serial.println(cfg.type);
            return NAN;
    }
}

float SensorManager::readRtdSensor() {
    uint8_t status = rtd.read_all();
    if (status == 0) {
        return rtd.temperature(); // °C
    } else {
        Serial.print("RTD fault register: ");
        Serial.println(status);
        return NAN; 
    }
}

float SensorManager::readDallasSensor() {
    dallasTemp.requestTemperatures();
    float temp = dallasTemp.getTempCByIndex(0);
    if (temp == DEVICE_DISCONNECTED_C) {
        Serial.println("Error: No se pudo leer el DS18B20.");
        return NAN;
    }
    return temp;
}

float SensorManager::readSensorValue(const SensorConfig &cfg) {
    switch(cfg.type) {
        case NTC_100K_TEMPERATURE_SENSOR:
        case NTC_10K_TEMPERATURE_SENSOR:
        case WATER_NTC_10K_TEMPERATURE_SENSOR:
        case PH_SENSOR:
        case CONDUCTIVITY_SENSOR: 
        case SOIL_HUMIDITY_SENSOR:
        case CONDENSATION_HUMIDITY_SENSOR:
        case VOLTAGE_SENSOR:
            return readAnalogSensor(cfg);
        case RTD_TEMPERATURE_SENSOR:
            return readRtdSensor();
        case DS18B20_TEMPERATURE_SENSOR:
            return readDallasSensor();
        default:
            return 0.0;
    }
}

SensorReading SensorManager::getSensorReading(const SensorConfig &cfg) {
    SensorReading reading;
    strncpy(reading.sensorId, cfg.sensorId, sizeof(reading.sensorId) - 1);
    strncpy(reading.sensorName, cfg.sensorName, sizeof(reading.sensorName) - 1);

    reading.type = cfg.type;
    reading.timestamp = rtcManager.getEpochTime();

    // Si es un sensor analógico y no tenemos lectura válida del ADC, actualizar
    if ((cfg.type == NTC_100K_TEMPERATURE_SENSOR || 
         cfg.type == NTC_10K_TEMPERATURE_SENSOR || 
         cfg.type == WATER_NTC_10K_TEMPERATURE_SENSOR||
         cfg.type == PH_SENSOR || 
         cfg.type == CONDUCTIVITY_SENSOR || 
         cfg.type == SOIL_HUMIDITY_SENSOR || 
         cfg.type == CONDENSATION_HUMIDITY_SENSOR || 
         cfg.type == VOLTAGE_SENSOR) && 
        !adcReadingValid) {
        updateADCReadings(1000);
    }

    reading.value = readSensorValue(cfg);

    // Debug print
    Serial.print("Lectura sensor - ID: ");
    Serial.print(reading.sensorId);
    Serial.print(" | Nombre: ");
    Serial.print(reading.sensorName);
    Serial.print(" | Valor: ");
    Serial.print(reading.value, 2);
    Serial.print(" | Timestamp: ");
    Serial.println(reading.timestamp);

    return reading;
}