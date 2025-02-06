/*******************************************************************************************
 * Archivo: src/main.cpp
 * Descripción: Código principal para el ESP32 que configura los sensores, radio LoRa, BLE y
 * entra en modo Deep Sleep. Se incluye inicialización de hardware y manejo de configuraciones.
 *******************************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <vector>
#include <ArduinoJson.h>
#include <cmath>  // Agregar para usar pow() y round()

// Bibliotecas específicas del proyecto
#include "config.h"
#include "PowerManager.h"
#include "MAX31865.h"
#include <RadioLib.h>
#include "RTCManager.h"
#include "sensor_types.h"
#include "SensirionI2cSht4x.h"
#include "SensorManager.h"
#include "ADS131M08.h"
#include "LoRaWAN_ESP32.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include "config_manager.h"
#include "ble_config_callbacks.h"
#include "utilities.h"
#include "ble_service.h"  // Incluir nuestro nuevo header de BLE

/*-------------------------------------------------------------------------------------------------
   Declaración de funciones
-------------------------------------------------------------------------------------------------*/
/**
 * @brief Configura el servicio BLE y sus características.
 * @param pServer Puntero al servidor BLE.
 * @return Puntero al servicio BLE creado.
 */
BLEService* setupBLEService(BLEServer* pServer);

/**
 * @brief Configura el ESP32 para entrar en deep sleep.
 */
void goToDeepSleep();

/**
 * @brief Verifica si se mantuvo presionado el botón de configuración y activa el modo BLE.
 */
void checkConfigMode();

/**
 * @brief Inicializa el bus I2C, la expansión de I/O y el PowerManager.
 */
void initHardware();

/**
 * @brief Envía el payload de sensores fragmentado para no superar el tamaño máximo permitido.
 *        Se limita la precisión a 6 decimales en cada medición y se incluye la cabecera en cada fragmento.
 * @param readings Vector con todas las lecturas de sensores.
 */
void sendFragmentedPayload(const std::vector<SensorReading>& readings);

/*-------------------------------------------------------------------------------------------------
   Objetos Globales y Variables
-------------------------------------------------------------------------------------------------*/

const LoRaWANBand_t Region = US915;
const uint8_t subBand = 2;  // For US915, change this to 2, otherwise leave on 0

Preferences preferences;       // Almacenamiento de preferencias en NVS

uint32_t timeToSleep;          // Tiempo en segundos para deep sleep
String deviceId;
String stationId;
bool systemInitialized;        // Variable global para la inicialización del sistema

RTCManager rtcManager;
PCA9555 ioExpander(I2C_ADDRESS_PCA9555, I2C_SDA_PIN, I2C_SCL_PIN);
PowerManager powerManager(ioExpander);

SPIClass spi(FSPI);
SPISettings spiAdcSettings(SPI_ADC_CLOCK, MSBFIRST, SPI_MODE1);
SPISettings spiRtdSettings(SPI_RTD_CLOCK, MSBFIRST, SPI_MODE1);
SPISettings spiRadioSettings(SPI_RADIO_CLOCK, MSBFIRST, SPI_MODE0);

ADS131M08 adc(ioExpander, spi, spiAdcSettings);
MAX31865_RTD rtd(MAX31865_RTD::RTD_PT100, spi, spiRtdSettings, ioExpander, PT100_CS_PIN);

SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, spi, spiRadioSettings);
LoRaWANNode node(&radio, &Region, subBand);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dallasTemp(&oneWire);

/*-------------------------------------------------------------------------------------------------
   Implementación de Funciones
-------------------------------------------------------------------------------------------------*/

/**
 * @brief Entra en modo deep sleep después de apagar periféricos y poner en reposo el módulo LoRa.
 */
void goToDeepSleep() {
    Serial.println("Entrando en Deep Sleep...");
    Serial.flush();
    
    // Configurar el temporizador y GPIO para despertar
    esp_sleep_enable_timer_wakeup(timeToSleep * 1000000ULL);
    gpio_wakeup_enable((gpio_num_t)CONFIG_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_deep_sleep_enable_gpio_wakeup(BIT(CONFIG_PIN), ESP_GPIO_WAKEUP_GPIO_LOW);
    
    // Apagar todos los reguladores y módulos innecesarios
    powerManager.allPowerOff();
    radio.sleep(false);
    btStop();
    Wire.end();
    spi.end();
    
    esp_deep_sleep_start();
}

/**
 * @brief Comprueba si se ha activado el modo configuración mediante un pin.
 *        Si se mantiene presionado el botón de configuración durante el tiempo definido, activa BLE.
 */
void checkConfigMode() {
    if (digitalRead(CONFIG_PIN) == LOW) {
        unsigned long startTime = millis();
        while (digitalRead(CONFIG_PIN) == LOW) {
            if (millis() - startTime >= CONFIG_TRIGGER_TIME) {

                // Inicializar BLE y crear servicio de configuración usando la nueva función modularizada
                LoRaConfig loraConfig = ConfigManager::getLoRaConfig();
                String bleName = "SENSOR_DEV" + String(loraConfig.devAddr);
                BLEDevice::init(bleName.c_str());

                BLEServer* pServer = BLEDevice::createServer();
                pServer->setCallbacks(new MyBLEServerCallbacks());  // Añadido el callback del servidor
                BLEService* pService = setupBLEService(pServer);

                // Configurar publicidad BLE
                BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
                pAdvertising->addServiceUUID(pService->getUUID());
                pAdvertising->setScanResponse(true);
                pAdvertising->setMinPreferred(0x06);
                pAdvertising->setMinPreferred(0x12);
                pAdvertising->start();

                // Bucle de parpadeo del LED de configuración
                while (true) {
                    ioExpander.digitalWrite(CONFIG_LED_PIN, HIGH);
                    delay(500);
                    ioExpander.digitalWrite(CONFIG_LED_PIN, LOW);
                    delay(500);
                }
            }
        }
    }
}

/**
 * @brief Inicializa configuraciones básicas de hardware:
 *        - Inicia el bus I2C.
 *        - Inicializa el expansor de I/O (PCA9555).
 *        - Configura el PowerManager.
 */
void initHardware() {
    // Inicializar I2C con pines definidos
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // Inicializar PCA9555
    if (!ioExpander.begin()) {
        Serial.println("Error al inicializar PCA9555");
    }

    // Inicializar PowerManager
    if (!powerManager.begin()) {
        Serial.println("Error al inicializar PowerManager");
    }
}

/*-------------------------------------------------------------------------------------------------
   Función setup()
   Inicializa la comunicación serial, hardware, configuración de sensores y radio,
   y recupera configuraciones previas del sistema.
-------------------------------------------------------------------------------------------------*/
void setup() {
    Serial.begin(115200);
    pinMode(CONFIG_PIN, INPUT);

    // Inicializar configuración por defecto en la primera ejecución
    if (!ConfigManager::checkInitialized()) {
        Serial.println("Primera ejecución detectada. Inicializando configuración...");
        ConfigManager::initializeDefaultConfig();
    }
    
    // Obtener la configuración del sistema, incluyendo sleepTime, deviceId y stationId
    ConfigManager::getSystemConfig(systemInitialized, timeToSleep, deviceId, stationId);
    
    // Verificar si se ha activado el modo configuración antes de continuar
    checkConfigMode();
    
    // // Inicialización del NVS y de hardware I2C/IO
    // preferences.clear();
    // nvs_flash_erase();
    // nvs_flash_init();

    // Inicializar hardware (I2C, PCA9555, PowerManager)
    initHardware();

    // Inicializar RTC
    if (!rtcManager.begin()) {
        Serial.println("No se pudo encontrar el RTC");
    }

    // Encender la alimentación requerida
    powerManager.power3V3On();
    powerManager.power2V5On();

    // Inicializar sensores
    SensorManager::beginSensors();
    
    // Inicializar radio LoRa y configurar datarate máximo para mayor capacidad de payload
    int16_t state = radio.begin();

    // Configurar parámetros de LoRa en modo ABP
    LoRaConfig loraConfig = ConfigManager::getLoRaConfig();
    uint8_t fNwkSIntKey[16], sNwkSIntKey[16], nwkSEncKey[16], appSKey[16];
    parseKeyString(loraConfig.fNwkSIntKey, fNwkSIntKey, 16);
    parseKeyString(loraConfig.sNwkSIntKey, sNwkSIntKey, 16);
    parseKeyString(loraConfig.nwkSEncKey,  nwkSEncKey,  16);
    parseKeyString(loraConfig.appSKey,     appSKey,     16);

    node.beginABP(loraConfig.devAddr, fNwkSIntKey, sNwkSIntKey, nwkSEncKey, appSKey);
    node.activateABP();
    node.setDatarate(4);  // DR4: SF8/BW500 ofrece buen balance entre alcance y tamaño de payload

    // Configurar pin para LED de configuración en el expansor
    ioExpander.pinMode(CONFIG_LED_PIN, OUTPUT);
}

/*-------------------------------------------------------------------------------------------------
   Función loop()
   Ejecuta el ciclo principal: comprobación del modo configuración, lecturas de sensores,
   impresión de resultados de depuración y finalmente entra en deep sleep.
-------------------------------------------------------------------------------------------------*/
void loop() {
    // Comprobar constantemente si se solicita el modo configuración
    checkConfigMode();
    
    // Obtener toda la configuración de sensores
    auto allSensors = ConfigManager::getAllSensorConfigs();
    
    // Añadir log para mostrar todos los sensores y su estado (habilitado/deshabilitado)
    Serial.println("GET ALL SENSOR: Se han obtenido las siguientes configuraciones de sensores:");
    for (size_t i = 0; i < allSensors.size(); i++) {
        Serial.printf("Sensor %zu - ID: %s - %s\n", i, allSensors[i].sensorId, allSensors[i].enable ? "HABILITADO" : "DESHABILITADO");
    }
    
    // Filtrar solo los sensores habilitados para su lectura
    std::vector<SensorConfig> enabledSensors;
    for (const auto& sensor : allSensors) {
        if (sensor.enable && strlen(sensor.sensorId) > 0) {
            enabledSensors.push_back(sensor);
        }
    }
    
    // Array para almacenar las lecturas de sensores
    std::vector<SensorReading> readings;
    
    // Forzar lectura inicial del ADC, timeout 1000ms
    SensorManager::updateADCReadings(1000);
    
    // Obtener lecturas de los sensores habilitados
    for (const auto& sensor : enabledSensors) {
        readings.push_back(SensorManager::getSensorReading(sensor));
    }
    
    // Enviar el payload fragmentado
    sendFragmentedPayload(readings);
    
    // Entrar en modo deep sleep tras finalizar las tareas del ciclo
    goToDeepSleep();
}

/**
 * @brief Redondea un valor a un número específico de decimales, solo si el valor 
 *        realmente tiene más decimales que el límite indicado.
 * @param value Valor a redondear.
 * @param decimals Número máximo de decimales permitidos.
 * @return Valor redondeado o el valor original si ya tiene la precisión requerida.
 */
double roundValue(double value, int decimals) {
    double factor = pow(10.0, decimals);
    double rounded = round(value * factor) / factor;
    
    // Si la diferencia es insignificante, se entiende que el valor no tenía más decimales
    if (fabs(rounded - value) < 1e-9) {  
        return value;
    }
    return rounded;
}

/**
 * @brief Envía el payload de sensores fragmentado para no superar el tamaño máximo permitido.
 *        Se limita la precisión a 6 decimales en cada medición y se incluye la cabecera en cada fragmento.
 * @param readings Vector con todas las lecturas de sensores.
 */
void sendFragmentedPayload(const std::vector<SensorReading>& readings) {
    // Obtener el tamaño máximo del payload permitido por la configuración LoRaWAN.
    const int MAX_PAYLOAD = 200; // DR4 max payload is 250 bytes
    size_t sensorIndex = 0;
    int fragmentNumber = 0;
    
    while (sensorIndex < readings.size()) {
        // Crear un nuevo payload con cabecera
        StaticJsonDocument<512> payload;
        payload["st"] = stationId;
        payload["d"] = deviceId;
        payload["vt"] = roundValue(SensorManager::readBatteryVoltage(), 6);
        payload["ts"] = rtcManager.getEpochTime();
        JsonArray sensorArray = payload.createNestedArray("s");
        
        String fragmentStr;
        // Agregar lecturas de sensores mientras no se exceda el tamaño máximo del payload
        while (sensorIndex < readings.size()) {
            // Limitar la precisión a 6 decimales usando la función roundValue
            double valorRedondeado = roundValue(readings[sensorIndex].value, 6);
            
            // Agregar la lectura al arreglo del payload
            JsonObject sensorObj = sensorArray.createNestedObject();
            sensorObj["id"] = readings[sensorIndex].sensorId;
            sensorObj["t"] = readings[sensorIndex].type;
            sensorObj["v"] = valorRedondeado;
            
            // Serializar para verificar el tamaño
            fragmentStr = "";
            serializeJson(payload, fragmentStr);
            
            // Si se excede el límite, eliminar la última lectura y salir del ciclo
            if (fragmentStr.length() > MAX_PAYLOAD) {
                sensorArray.remove(sensorArray.size() - 1);
                break;
            }
            
            sensorIndex++;
        }
        
        // Volver a serializar el payload final para este fragmento
        fragmentStr = "";
        serializeJson(payload, fragmentStr);
        Serial.printf("Enviando fragmento %d con tamaño %d bytes\n", fragmentNumber, fragmentStr.length());
        Serial.println(fragmentStr);
        
        int16_t state = node.sendReceive((uint8_t*)fragmentStr.c_str(), fragmentStr.length());
        if (state == RADIOLIB_ERR_NONE) {
            Serial.printf("Fragmento %d enviado correctamente\n", fragmentNumber);
        } else {
            Serial.printf("Error enviando fragmento %d: %d\n", fragmentNumber, state);
        }
        
        fragmentNumber++;
    }
}

