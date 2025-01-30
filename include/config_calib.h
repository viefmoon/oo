// Configuración de calibración predeterminada para todos los sensores
#pragma once

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

// =============== Calibración sensor conductividad ===============
// Valores por defecto para calibración TDS (ajustar según necesidades)
#define CONDUCTIVITY_DEFAULT_V1    0.5f    // Voltaje en solución baja
#define CONDUCTIVITY_DEFAULT_T1    200.0f  // TDS en ppm
#define CONDUCTIVITY_DEFAULT_V2    1.0f    // Voltaje en solución media
#define CONDUCTIVITY_DEFAULT_T2    1000.0f
#define CONDUCTIVITY_DEFAULT_V3    1.5f    // Voltaje en solución alta
#define CONDUCTIVITY_DEFAULT_T3    2000.0f
#define TEMP_COEF_COMPENSATION     0.02f    // Compensación térmica por 2% /°C
#define CONDUCTIVITY_DEFAULT_TEMP         25.0f    // Temperatura de calibración

// =============== Calibración pH ===============
#define PH_DEFAULT_V1          0.4425         // Voltaje a pH bajo (4.01)
#define PH_DEFAULT_T1          4.01        // Valor de pH bajo
#define PH_DEFAULT_V2          0.001         // Voltaje a pH medio (6.86)
#define PH_DEFAULT_T2          6.86        // Valor de pH medio
#define PH_DEFAULT_V3          -0.32155        // Voltaje a pH alto (9.18)
#define PH_DEFAULT_T3          9.18        // Valor de pH alto
#define PH_DEFAULT_TEMP        25.0        // Temperatura de calibración por defecto