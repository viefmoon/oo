#include <Arduino.h>
#include "config.h"
#include "PowerManager.h"
#include "Wire.h"
#include "SPI.h"
#include "MAX31865.h"
#include <RadioLib.h>
#include "RTCManager.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "sensor_types.h"
#include "SensirionI2cSht4x.h"
#include "SensorManager.h"
#include "LoraConfig.h"
#include "ADS131M08.h"
#include "LoRaWAN_ESP32.h"
#include <Preferences.h>
#include "nvs_flash.h"
#include "esp_sleep.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include "config_manager.h"
#include "ble_config_callbacks.h"

// Declaración de función
BLEService* setupBLEService(BLEServer* pServer);

// ====== Objetos Globales ======
Preferences preferences;  // Declaración global de preferences
uint32_t frameCounter;   // Declaración global del contador
uint32_t timeToSleep;  // Declaración global del tiempo de sleep

RTCManager rtcManager;
PCA9555 ioExpander(I2C_ADDRESS_PCA9555, I2C_SDA_PIN, I2C_SCL_PIN);
PowerManager powerManager(ioExpander);

SPIClass spi(FSPI);
SPISettings spiAdcSettings(100000, MSBFIRST, SPI_MODE1);
SPISettings spiRtdSettings(1000000, MSBFIRST, SPI_MODE1);
SPISettings spiRadioSettings(100000, MSBFIRST, SPI_MODE0);

ADS131M08 adc(ioExpander, spi, spiAdcSettings);

MAX31865_RTD rtd(MAX31865_RTD::RTD_PT100, spi, spiRtdSettings, ioExpander, PT100_CS_PIN);

SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, spi, spiRadioSettings);
LoRaWANNode node(&radio, &Region, subBand);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dallasTemp(&oneWire);

// ====== Declaración de sensores (ejemplo) ======
SensorConfig sensorConfigs[] = {
    // --- ADC (8 canales) ---
    {"NTC0", "Temperatura_NTC0", NTC_100K_TEMPERATURE_SENSOR, 1, 0, "", false},
    {"NTC1", "Temperatura_NTC1", NTC_100K_TEMPERATURE_SENSOR, 1, 1, "", false},
    {"HDS10", "Humedad_HDS10", CONDENSATION_HUMIDITY_SENSOR, 1, 2, "", false},
    {"SUELO1", "Humedad_Suelo1", SOIL_HUMIDITY_SENSOR, 1, 3, "", false},
    {"SUELO2", "Humedad_Suelo2", SOIL_HUMIDITY_SENSOR, 1, 4, "", false},
    {"CONDUCTIVITY", "Conductividad_Agua", CONDUCTIVITY_SENSOR, 1, 5, "NTC0", false},
    {"BATT", "Voltaje_Bateria", VOLTAGE_SENSOR, 1, 6, "", false},
    {"PH", "PH_Sensor", PH_SENSOR, 1, 7, "NTC0", false},
    
    // --- RTD (PT100) ---
    {"PT100", "Temperatura_PT100", RTD_TEMPERATURE_SENSOR, 0, 0, "", false},
    
    // --- DS18B20 ---
    {"DS18B20", "Temperatura_DS18B20", DS18B20_TEMPERATURE_SENSOR, 0, 0, "", false}
};

// Para calcular el número de sensores
const size_t NUM_SENSORS = sizeof(sensorConfigs) / sizeof(sensorConfigs[0]);

/**
 * @brief Configura el ESP32 para entrar en deep sleep por TIME_TO_SLEEP_SECONDS.
 */
void goToDeepSleep() {
    Serial.println("Entrando en Deep Sleep...");
    Serial.flush();
    
    esp_sleep_enable_timer_wakeup(timeToSleep * 1000000ULL);
    
    // Configurar el tipo de interrupción para el pin de configuración
    gpio_wakeup_enable((gpio_num_t)CONFIG_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_deep_sleep_enable_gpio_wakeup(BIT(CONFIG_PIN), ESP_GPIO_WAKEUP_GPIO_LOW);
    
    // Apagar todos los reguladores
    powerManager.allPowerOff();
    
    // Poner el módulo LoRa en modo sleep (cold start)
    radio.sleep(false);
    
    btStop();
    Wire.end();
    spi.end();
    
    // Entrar en deep sleep
    esp_deep_sleep_start();
}

void checkConfigMode() {
    if (digitalRead(CONFIG_PIN) == LOW) {
        unsigned long startTime = millis();
        
        while (digitalRead(CONFIG_PIN) == LOW) {
            if (millis() - startTime >= CONFIG_TRIGGER_TIME) {
                Serial.println("Modo configuración activado");
                
                // Inicializar preferences aquí para asegurar que está abierto
                preferences.begin("lorawan", false);
                
                // Inicializar BLE
                BLEDevice::init("ESP32_Config");
                BLEServer *pServer = BLEDevice::createServer();
                
                // Crear servicio BLE y obtener referencia
                BLEService *pService = setupBLEService(pServer);
                
                // Configurar advertising
                BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
                pAdvertising->addServiceUUID(pService->getUUID()); // Usar el servicio creado directamente
                pAdvertising->setScanResponse(true);
                pAdvertising->setMinPreferred(0x06);  // funciones que ayudan con iPhone conexiones problema
                pAdvertising->setMinPreferred(0x12);
                pAdvertising->start();
                
                Serial.println("BLE activado - Usa una app para leer/escribir el tiempo de sleep");
                
                // Bucle de parpadeo
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

BLEService* setupBLEService(BLEServer* pServer) {
    // Crear servicio de configuración
    BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x180A));

    // Característica para tiempo de sleep
    BLECharacteristic *pSleepChar = pService->createCharacteristic(
        BLEUUID("2A37"),  // UUID personalizado para sleep time
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pSleepChar->setCallbacks(new SleepConfigCallback());

    // Característica para configuración NTC 100K
    BLECharacteristic *pNTC100KChar = pService->createCharacteristic(
        BLEUUID("2A38"),  // UUID personalizado para NTC 100K
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pNTC100KChar->setCallbacks(new NTC100KConfigCallback());

    // Característica para configuración NTC 10K
    BLECharacteristic *pNTC10KChar = pService->createCharacteristic(
        BLEUUID("2A39"),  // UUID personalizado para NTC 10K
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pNTC10KChar->setCallbacks(new NTC10KConfigCallback());

    // Característica para configuración de conductividad
    BLECharacteristic *pCondChar = pService->createCharacteristic(
        BLEUUID("2A3C"),  // UUID personalizado para conductividad
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pCondChar->setCallbacks(new ConductivityConfigCallback());

    // Característica para configuración de pH
    BLECharacteristic *pPHChar = pService->createCharacteristic(
        BLEUUID("2A3B"),  // UUID personalizado para pH
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pPHChar->setCallbacks(new PHConfigCallback());

    BLECharacteristic *pSensorsChar = pService->createCharacteristic(
    BLEUUID("2A40"), // UUID arbitrario para la config de sensores
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pSensorsChar->setCallbacks(new SensorsConfigCallback());

    pService->start();
    return pService;
}

// ====== setup() ======
void setup()
{
    Serial.begin(115200);
    pinMode(CONFIG_PIN, INPUT);
    
    // Verificar si es primera ejecución y sembrar valores por defecto
    if (!ConfigManager::checkInitialized()) {
        Serial.println("Primera ejecución detectada. Inicializando configuración...");
        ConfigManager::initializeDefaultConfig();
    }
    
    // Verificar modo configuración antes de cualquier otra inicialización
    checkConfigMode();

    // Borrar todas las preferencias almacenadas
    // preferences.clear();
    // nvs_flash_erase();
    // nvs_flash_init();
    // Inicializar I2C y PCA9555
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // Inicializar PCA9555
    if (!ioExpander.begin()) {
        Serial.println("Error al inicializar PCA9555");
    }

    // Inicializar PowerManager
    if (!powerManager.begin()) {
        Serial.println("Error al inicializar PowerManager");
    }

    // Inicializar RTC
    if (!rtcManager.begin()) {
        Serial.println("No se pudo encontrar el RTC");
    }

    // Encender alimentación 3.3V, 2.5V Y -2.5V
    powerManager.power3V3On();
    powerManager.power2V5On();
    
    // Aquí llamamos a la inicialización de todos los sensores
    SensorManager::beginSensors();

    int16_t state = radio.begin();
    debug(state != RADIOLIB_ERR_NONE, F("Initialise radio failed"), state, true);
    // Inicializar preferencias y recuperar el último frame counter
    preferences.begin("lorawan", false);
    frameCounter = preferences.getUInt("fcnt", 0);
    timeToSleep = preferences.getUInt("sleep_time", DEFAULT_TIME_TO_SLEEP);
    preferences.end(); // Cerrar preferences después de leer

    node.beginABP(devAddr, fNwkSIntKey, sNwkSIntKey, nwkSEncKey, appSKey);
    node.activateABP(); 
    debug(state != RADIOLIB_ERR_NONE, F("Activate ABP failed"), state, true);

    // // Setup the OTAA session information
    // node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
    // Serial.println(F("Join ('login') the LoRaWAN Network"));
    // state = node.activateOTAA();
    // debug(state != RADIOLIB_LORAWAN_NEW_SESSION, F("Join failed"), state, true);

    Serial.println(F("Ready!\n"));
        Serial.println(F("sleep_time: "));
    Serial.println(timeToSleep);

    // Después de inicializar el expansor I/O, configurar el pin del LED como salida
    ioExpander.pinMode(CONFIG_LED_PIN, OUTPUT);
}

void loop() {
    // Verificar modo configuración al inicio de cada ciclo
    checkConfigMode();
    
    // Crear array para almacenar lecturas
    SensorReading readings[NUM_SENSORS];
    
    // Forzar una lectura del ADC antes de comenzar
    SensorManager::updateADCReadings();
    
    // Leer todos los sensores
    for(size_t i = 0; i < NUM_SENSORS; i++) {
        readings[i] = SensorManager::getSensorReading(sensorConfigs[i]);
    }

    // // Construir payload LoRaWAN
    // String payload;
    // for(size_t i = 0; i < NUM_SENSORS; i++) {
    //     payload += readings[i].sensorId;
    //     payload += "|";
    //     payload += readings[i].sensorName;
    //     payload += "|";
    //     payload += String(readings[i].type);
    //     payload += "|";
    //     payload += String(readings[i].value, 4);
    //     payload += ";";
    // }
    
    // // Convertir String a byte array
    // uint8_t uplinkPayload[payload.length() + 1];
    // payload.getBytes(uplinkPayload, sizeof(uplinkPayload));
    
    // Serial.println(F("Enviando datos por LoRaWAN..."));
    
    // // Crear estructura para el evento
    // LoRaWANEvent_t event;
    
    // // Realizar el envío
    // int state = node.sendReceive(uplinkPayload, sizeof(uplinkPayload), 1, false, &event);    
    
    // if (state == RADIOLIB_ERR_NONE || state == RADIOLIB_LORAWAN_NO_DOWNLINK) {
    //     frameCounter = event.fCnt;
    //     preferences.putUInt("fcnt", frameCounter);
    //     Serial.print("Frame Counter: ");
    //     Serial.println(frameCounter);
    // }
    
    // debug((state != RADIOLIB_LORAWAN_NO_DOWNLINK) && (state != RADIOLIB_ERR_NONE), 
    //       F("Error in sendReceive"), state, false);
    
    // Después de enviar los datos, ir a deep sleep
    goToDeepSleep();
}

