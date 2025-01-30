#ifndef CONFIG_H
#define CONFIG_H

// Configuración de pines
#define ONE_WIRE_BUS        19    
#define I2C_SDA_PIN         1
#define I2C_SCL_PIN         0

//Direcciones i2c
#define I2C_ADDRESS_PCA9555 0x20

// Definiciones de pines SPI
#define SPI_SCK_PIN     10    // Pin de reloj SPI
#define SPI_MISO_PIN    6     // Pin MISO (Master In Slave Out)
#define SPI_MOSI_PIN    7     // Pin MOSI (Master Out Slave In)

// Otras configuraciones
#define SERIAL_BAUD     115200       // Velocidad del puerto serial

// Configuración Deep Sleep
#define DEFAULT_TIME_TO_SLEEP   30       // Tiempo de sueño en segundos (5 minutos = 300 segundos)

// Definiciones para pt100
#define PT100_CS_PIN P04        // Fuente de 3.3V para sensores en IO0_0

// Definiciones para PowerManager
#define POWER_3V3_PIN P00        // Fuente de 3.3V para sensores en IO0_0
#define POWER_12V_PIN P01        // Fuente de 12V en IO0_1
#define POWER_2V5_PIN P02        // Fuente de 2.5V y -2.5en IO0_2
#define POWER_STABILIZE_DELAY 100 // Tiempo de estabilización en ms

// Definiciones de pines ADC m04
#define ADC1_CS_PIN    P05
#define ADC1_DRDY_PIN  P06
#define ADC1_RST_PIN   P07
#define ADC2_CS_PIN    P10
#define ADC2_DRDY_PIN  P11
#define ADC2_RST_PIN   P12

// Definiciones de pines ADC m08
#define ADC_CS_PIN    P05
#define ADC_DRDY_PIN  P06
#define ADC_RST_PIN   P13

// Definiciones de pines para FlowSensor
#define FLOW_SENSOR_PIN P14

// Configuración del modo Config
#define CONFIG_PIN          2     // Pin para entrar en modo config
#define CONFIG_TRIGGER_TIME 5000  // 5 segundos para activar
#define CONFIG_TIMEOUT      30000 // 30 segundos de timeout
#define CONFIG_LED_PIN P11

// Pines para el LoRa 
#define LORA_NSS_PIN   8   // LoRa NSS
#define LORA_BUSY_PIN  4   // LoRa BUSY
#define LORA_RST_PIN   5   // LoRa RST
#define LORA_DIO1_PIN  3   // LoRa DIO1

// Definiciones de pines para PT100
#endif // CONFIG_H 