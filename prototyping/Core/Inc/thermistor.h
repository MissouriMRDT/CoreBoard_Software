/*
 * thermistor.h
 *
 *  Created on: Nov 5, 2026
 *      Author: lukero45
 */

#ifndef INC_THERMISTOR_H_
#define INC_THERMISTOR_H_
// Core/Inc/thermistor.h


#include <stdint.h>

// Data Structures: Holds all variables related to the thermistor
#define THERMISTOR_ADC_MAX   4095.0f
#define THERMISTOR_VREF      3.3f
#define RESISTOR_DIV         10000.0f

// Thermistor Datasheet Constants
#define THERMISTOR_R25          10000.0f  // Nominal resistance at 25°C
#define THERMISTOR_BETA         3950.0f   // Beta coefficient (check datasheet)
#define THERMISTOR_T25_K        298.15f   // 25°C in Kelvin

typedef struct {
    uint32_t raw_adc;      // Raw reading from PA4
    float temperature_c;   // Calculated temperature in °C
} Thermistor_t;

// Function Declarations
void Thermistor_Init(Thermistor_t *dev);
float Thermistor_ConvertRawToTempC(uint32_t raw_adc);
void Thermistor_Update(Thermistor_t *dev, uint32_t raw_adc);

#endif /* INC_THERMISTOR_H_ */
