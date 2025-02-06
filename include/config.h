#ifndef CONFIG_H
#define CONFIG_H

// ==========================================================
//              DEFINICIONES FIXAS DE HARDWARE
// ==========================================================

// Configuración de pines generales
#define ONE_WIRE_BUS        19    
#define I2C_SDA_PIN         1
#define I2C_SCL_PIN         0

// Direcciones I2C
#define I2C_ADDRESS_PCA9555 0x20

// Pines para SPI
#define SPI_SCK_PIN     10    // Pin de reloj SPI
#define SPI_MISO_PIN    6     // Pin MISO (Master In Slave Out)
#define SPI_MOSI_PIN    7     // Pin MOSI (Master Out Slave In)

// Pines para pt100 y PowerManager
#define PT100_CS_PIN    P04        // Fuente de 3.3V para sensores en IO0_0
#define POWER_3V3_PIN   P00        // Fuente de 3.3V para sensores en IO0_0
#define POWER_12V_PIN   P01        // Fuente de 12V en IO0_1
#define POWER_2V5_PIN   P02        // Fuente de 2.5V y -2.5 en IO0_2
#define POWER_STABILIZE_DELAY 100   // Tiempo de estabilización en ms

// Pines para ADC m04
#define ADC1_CS_PIN    P05
#define ADC1_DRDY_PIN  P06
#define ADC1_RST_PIN   P07
#define ADC2_CS_PIN    P10
#define ADC2_DRDY_PIN  P11
#define ADC2_RST_PIN   P12

// Pines para ADC m08
#define ADC_CS_PIN    P05
#define ADC_DRDY_PIN  P06
#define ADC_RST_PIN   P13

// Pin para FlowSensor
#define FLOW_SENSOR_PIN P14

// Definición del canal ADC para la lectura del voltaje de la batería
#define BATTERY_ADC_CHANNEL 6

// Pines para modo Config
#define CONFIG_PIN          2     // Pin para entrar en modo config
#define CONFIG_TRIGGER_TIME 5000  // 5 segundos para activar
#define CONFIG_TIMEOUT      30000 // 30 segundos de timeout
#define CONFIG_LED_PIN      P11

// Pines para LoRa
#define LORA_NSS_PIN    8   // LoRa NSS
#define LORA_BUSY_PIN   4   // LoRa BUSY
#define LORA_RST_PIN    5   // LoRa RST
#define LORA_DIO1_PIN   3   // LoRa DIO1

// ==========================================================
//              CONFIGURACIONES MODIFICABLES
// ==========================================================

// Configuración del puerto serial
#define SERIAL_BAUD     115200       // Velocidad del puerto serial

// Configuración Deep Sleep
#define DEFAULT_TIME_TO_SLEEP   30       // Tiempo de sueño en segundos 

// ID por defecto del dispositivo
#define DEFAULT_DEVICE_ID "DEVICE_001"
#define DEFAULT_STATION_ID "STATION_001"

// ==========================================================
//                  CALIBRACIONES
// ==========================================================

// =============== Calibración NTC 100K ===============
#define DEFAULT_T1_100K        25.0    // Temperatura 1 en °C
#define DEFAULT_R1_100K    100000.0    // Resistencia a T1
#define DEFAULT_T2_100K        35.0    // Temperatura 2 en °C
#define DEFAULT_R2_100K     64770.0    // Resistencia a T2
#define DEFAULT_T3_100K        45.0    // Temperatura 3 en °C
#define DEFAULT_R3_100K     42530.0    // Resistencia a T3

// =============== Calibración NTC 10K ================
#define DEFAULT_T1_10K         25.0    // Temperatura 1 en °C
#define DEFAULT_R1_10K      10000.0    // Resistencia a T1
#define DEFAULT_T2_10K         35.0    // Temperatura 2 en °C
#define DEFAULT_R2_10K       6477.0    // Resistencia a T2  
#define DEFAULT_T3_10K         45.0    // Temperatura 3 en °C
#define DEFAULT_R3_10K       4253.0    // Resistencia a T3

// =============== Calibración sensor de conductividad ===============
#define CONDUCTIVITY_DEFAULT_V1    0.5f    // Voltaje en solución baja
#define CONDUCTIVITY_DEFAULT_T1    200.0f  // TDS en ppm
#define CONDUCTIVITY_DEFAULT_V2    1.0f    // Voltaje en solución media
#define CONDUCTIVITY_DEFAULT_T2    1000.0f
#define CONDUCTIVITY_DEFAULT_V3    1.5f    // Voltaje en solución alta
#define CONDUCTIVITY_DEFAULT_T3    2000.0f
#define TEMP_COEF_COMPENSATION     0.02f   // Compensación térmica por 2% /°C
#define CONDUCTIVITY_DEFAULT_TEMP  25.0f   // Temperatura de calibración

// =============== Calibración pH ===============
#define PH_DEFAULT_V1          0.4425   // Voltaje a pH bajo (4.01)
#define PH_DEFAULT_T1          4.01     // Valor de pH bajo
#define PH_DEFAULT_V2          0.001    // Voltaje a pH medio (6.86)
#define PH_DEFAULT_T2          6.86     // Valor de pH medio
#define PH_DEFAULT_V3          -0.32155 // Voltaje a pH alto (9.18)
#define PH_DEFAULT_T3          9.18     // Valor de pH alto
#define PH_DEFAULT_TEMP        25.0     // Temperatura de calibración por defecto

// ==========================================================
//           CONFIGURACIÓN DE LoRa 
// ==========================================================
#define DEFAULT_LORA_DEVADDR     0x00bfe104
#define DEFAULT_FNWKS_INTKEY     "63,67,7A,0E,C1,28,7B,82,92,C5,0B,A1,3D,D6,D2,25"
#define DEFAULT_SNWKS_INTKEY     "DD,14,59,AB,63,6E,50,0D,8A,75,77,A8,75,28,C9,01"
#define DEFAULT_NWK_SENCKEY      "64,13,E3,99,91,F0,87,56,99,C9,BD,8C,72,07,8F,9E"
#define DEFAULT_APPS_KEY         "EE,F1,30,98,6A,11,4E,69,D0,DE,8A,DC,D6,8D,28,A6"

// BLE service y características UUID
#define BLE_SERVICE_UUID             "180A"
#define BLE_CHAR_SYSTEM_UUID         "2A37"
#define BLE_CHAR_NTC100K_UUID        "2A38"
#define BLE_CHAR_NTC10K_UUID         "2A39"
#define BLE_CHAR_CONDUCTIVITY_UUID   "2A3C"
#define BLE_CHAR_PH_UUID             "2A3B"
#define BLE_CHAR_SENSORS_UUID        "2A40"
#define BLE_CHAR_LORA_CONFIG_UUID    "2A41"

// ==========================================================
//           CONFIGURACIÓN DE SPI
// ==========================================================

// Velocidad de SPI para el ADC m08 (por ejemplo, ADS131M08)
#define SPI_ADC_CLOCK     100000    // 100 kHz
// Velocidad de SPI para el sensor MAX31865 (RTD - PT100)
#define SPI_RTD_CLOCK     1000000   // 1 MHz
// Velocidad de SPI para la radio LoRa
#define SPI_RADIO_CLOCK   100000    // 100 kHz

// ==========================================================
//          CLAVES PARA CONFIGURACIÓN EN NVS (Preferences)
// ==========================================================

#define KEY_INITIALIZED        "initialized"
#define KEY_SLEEP_TIME         "sleep_time"
#define KEY_STATION_ID         "stationId"
#define KEY_DEVICE_ID          "deviceId"

#define KEY_VOLT               "volt"

// Configuración NTC 100K
#define KEY_NTC100K_T1         "n100k_t1"
#define KEY_NTC100K_R1         "n100k_r1"
#define KEY_NTC100K_T2         "n100k_t2"
#define KEY_NTC100K_R2         "n100k_r2"
#define KEY_NTC100K_T3         "n100k_t3"
#define KEY_NTC100K_R3         "n100k_r3"

// Configuración NTC 10K
#define KEY_NTC10K_T1         "n10k_t1"
#define KEY_NTC10K_R1         "n10k_r1"
#define KEY_NTC10K_T2         "n10k_t2"
#define KEY_NTC10K_R2         "n10k_r2"
#define KEY_NTC10K_T3         "n10k_t3"
#define KEY_NTC10K_R3         "n10k_r3"

// Configuración de Conductividad
#define KEY_CONDUCT_CT         "c_ct"
#define KEY_CONDUCT_CC         "c_cc"
#define KEY_CONDUCT_V1         "c_v1"
#define KEY_CONDUCT_T1         "c_t1"
#define KEY_CONDUCT_V2         "c_v2"
#define KEY_CONDUCT_T2         "c_t2"
#define KEY_CONDUCT_V3         "c_v3"
#define KEY_CONDUCT_T3         "c_t3"

// Configuración de pH
#define KEY_PH_V1              "ph_v1"
#define KEY_PH_T1              "ph_t1"
#define KEY_PH_V2              "ph_v2"
#define KEY_PH_T2              "ph_t2"
#define KEY_PH_V3              "ph_v3"
#define KEY_PH_T3              "ph_t3"
#define KEY_PH_CT              "ph_ct"

// --- Claves para la configuración de cada sensor individual ---
// Estas claves se utilizan dentro del arreglo "sensors"
#define KEY_SENSOR             "k"
#define KEY_SENSOR_ID          "id"
#define KEY_SENSOR_ID_TEMPERATURE_SENSOR   "ts"
#define KEY_SENSOR_TYPE        "t"
#define KEY_SENSOR_CHANNEL     "ch"
#define KEY_SENSOR_VALUE       "v"
#define KEY_SENSOR_ENABLE      "e"

// Configuración LoRa
#define KEY_LORA_DEVADDR       "devAddr"
#define KEY_LORA_FNWS_INTKEY   "fNwkSIntKey"
#define KEY_LORA_SNWS_INTKEY   "sNwkSIntKey"
#define KEY_LORA_NWKSENC_KEY   "nwkSEncKey"
#define KEY_LORA_APPS_KEY      "appSKey"

#define KEY_LORAWAN_SESSION    "lorawan_session"


// ==========================================================
//       NAMESPACES PARA CONFIGURACIÓN (NVS/Preferences)
// ==========================================================
#define NAMESPACE_SYSTEM    "system"

#define NAMESPACE_NTC100K "ntc_100k"
#define NAMESPACE_NTC10K "ntc_10k"
#define NAMESPACE_COND      "cond"
#define NAMESPACE_PH        "ph"
#define NAMESPACE_LORAWAN   "lorawan"
#define NAMESPACE_SENSORS   "sensors"

// ==========================================================
//            CLAVES ADICIONALES DE CONFIGURACIÓN
// ==========================================================
#define VALUE_INITIALIZED   true
#define KEY_FCNT            "fcnt"        // Agregado para el frame counter (fcnt)

#endif // CONFIG_H 