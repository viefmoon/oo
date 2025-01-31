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

// Bibliotecas específicas del proyecto
#include "config.h"
#include "PowerManager.h"
#include "MAX31865.h"
#include <RadioLib.h>
#include "RTCManager.h"
#include "sensor_types.h"
#include "SensirionI2cSht4x.h"
#include "SensorManager.h"
#include "LoraConfig.h"
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

/*-------------------------------------------------------------------------------------------------
   Objetos Globales y Variables
-------------------------------------------------------------------------------------------------*/
Preferences preferences;       // Almacenamiento de preferencias en NVS
uint32_t frameCounter;         // Contador de tramas enviadas
uint32_t timeToSleep;          // Tiempo en segundos para deep sleep

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
 *        Si el botón de configuración se mantiene presionado el tiempo definido, activa BLE.
 */
void checkConfigMode() {
    if (digitalRead(CONFIG_PIN) == LOW) {
        unsigned long startTime = millis();
        
        while (digitalRead(CONFIG_PIN) == LOW) {
            if (millis() - startTime >= CONFIG_TRIGGER_TIME) {
                Serial.println("Modo configuración activado");
                
                // Inicializar BLE y crear servicio de configuración
                // Se cambia el nombre del dispositivo BLE a "SENSOR_DEV" concatenado con loraConfig.devAddr
                LoRaConfig loraConfig = ConfigManager::getLoRaConfig();
                String bleName = "SENSOR_DEV" + String(loraConfig.devAddr);
                BLEDevice::init(bleName.c_str());
                
                BLEServer *pServer = BLEDevice::createServer();
                BLEService *pService = setupBLEService(pServer);
                
                // Configurar publicidad BLE
                BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
                pAdvertising->addServiceUUID(pService->getUUID());
                pAdvertising->setScanResponse(true);
                pAdvertising->setMinPreferred(0x06);
                pAdvertising->setMinPreferred(0x12);
                pAdvertising->start();
                
                Serial.println("BLE activado - Usa una app para leer/escribir el tiempo de sleep");
                
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
 * @brief Crea y configura el servicio BLE con sus respectivas características para configuración.
 * 
 * @param pServer Puntero al servidor BLE.
 * @return BLEService* Puntero al servicio creado.
 */
BLEService* setupBLEService(BLEServer* pServer) {
    // Crear servicio de configuración usando el UUID definido en config.h
    BLEService *pService = pServer->createService(BLEUUID(BLE_SERVICE_UUID));

    // Característica para tiempo de sleep
    BLECharacteristic *pSleepChar = pService->createCharacteristic(
        BLEUUID(BLE_CHAR_SLEEP_UUID),
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pSleepChar->setCallbacks(new SleepConfigCallback());

    // Característica para configuración NTC 100K
    BLECharacteristic *pNTC100KChar = pService->createCharacteristic(
        BLEUUID(BLE_CHAR_NTC100K_UUID),
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pNTC100KChar->setCallbacks(new NTC100KConfigCallback());

    // Característica para configuración NTC 10K
    BLECharacteristic *pNTC10KChar = pService->createCharacteristic(
        BLEUUID(BLE_CHAR_NTC10K_UUID),
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pNTC10KChar->setCallbacks(new NTC10KConfigCallback());

    // Característica para configuración de conductividad
    BLECharacteristic *pCondChar = pService->createCharacteristic(
        BLEUUID(BLE_CHAR_CONDUCTIVITY_UUID),
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pCondChar->setCallbacks(new ConductivityConfigCallback());

    // Característica para configuración de pH
    BLECharacteristic *pPHChar = pService->createCharacteristic(
        BLEUUID(BLE_CHAR_PH_UUID),
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pPHChar->setCallbacks(new PHConfigCallback());

    // Característica para configuración de sensores
    BLECharacteristic *pSensorsChar = pService->createCharacteristic(
        BLEUUID(BLE_CHAR_SENSORS_UUID),
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pSensorsChar->setCallbacks(new SensorsConfigCallback());

    // Característica para configuración LoRa
    BLECharacteristic *pLoRaConfigChar = pService->createCharacteristic(
        BLEUUID(BLE_CHAR_LORA_CONFIG_UUID),
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pLoRaConfigChar->setCallbacks(new LoRaConfigCallback());

    pService->start();
    return pService;
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
    
    // Verificar si se ha activado el modo configuración antes de continuar
    checkConfigMode();
    
    // Inicialización del NVS y de hardware I2C/IO
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
    
    // Inicializar radio LoRa
    int16_t state = radio.begin();
    debug(state != RADIOLIB_ERR_NONE, F("Initialise radio failed"), state, true);

    // Recuperar frame counter y tiempo de sleep desde configuración almacenada
    frameCounter = ConfigManager::getFrameCounter();
    timeToSleep  = ConfigManager::getSleepTime();

    // Configurar parámetros de LoRa en modo ABP
    LoRaConfig loraConfig = ConfigManager::getLoRaConfig();
    uint8_t fNwkSIntKey[16], sNwkSIntKey[16], nwkSEncKey[16], appSKey[16];
    parseKeyString(loraConfig.fNwkSIntKey, fNwkSIntKey, 16);
    parseKeyString(loraConfig.sNwkSIntKey, sNwkSIntKey, 16);
    parseKeyString(loraConfig.nwkSEncKey,  nwkSEncKey,  16);
    parseKeyString(loraConfig.appSKey,     appSKey,     16);

    node.beginABP(loraConfig.devAddr, fNwkSIntKey, sNwkSIntKey, nwkSEncKey, appSKey);
    node.activateABP();
    debug(state != RADIOLIB_ERR_NONE, F("Activate ABP failed"), state, true);

    // // Setup the OTAA session information
    // node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
    // Serial.println(F("Join ('login') the LoRaWAN Network"));
    // state = node.activateOTAA();
    // debug(state != RADIOLIB_LORAWAN_NEW_SESSION, F("Join failed"), state, true);

    Serial.println(F("Ready!\n"));

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
    
    // Recuperar la lista de sensores y filtrar los habilitados
    std::vector<SensorConfig> enabledSensors;
    auto allSensors = ConfigManager::getAllSensorConfigs();
    
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
    
    // Construir el payload JSON con la información del device y los sensores:
    StaticJsonDocument<1024> payload;
    payload["deviceId"] = ConfigManager::getDeviceId();
    payload["volt"] = SensorManager::readBatteryVoltage();

    JsonArray sensorsArray = payload.createNestedArray("s");
    for (const auto &reading : readings) {
        JsonObject sensorObj = sensorsArray.createNestedObject();
        sensorObj["id"] = reading.sensorId;
        sensorObj["tp"] = reading.type;
        sensorObj["v"] = reading.value;
        sensorObj["ts"] = reading.timestamp;
    }


    String payloadStr; 
    serializeJson(payload, payloadStr);
    Serial.println("Payload construido:");
    Serial.println(payloadStr);
    
    
    // Actualizar el frame counter en la configuración después de enviar datos
    ConfigManager::setFrameCounter(frameCounter);
    
    // Entrar en modo deep sleep tras finalizar las tareas del ciclo
    goToDeepSleep();
}

