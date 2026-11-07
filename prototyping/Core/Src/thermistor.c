/*
 * thermistor.c
 *
 *  Created on: Nov 5, 2026
 *      Author: luker
 */
#include "thermistor.h"

void Thermistor_Init(Thermistor_t *dev) { //Initializes the Thermistor
	if (dev == 0) return;
	dev->raw_adc = 0;
	dev->temperature_c = 0.0f;
}

float Thermistor_ConvertRawToTempC(uint32_t raw_adc){ // Converts the Raw ADC value to a Temp in C

	float voltage = ((float)raw_adc /THERMISTOR_ADC_MAX) * THERMISTOR_VREF; //Calculate Voltage across Thermistor
	float resistance = RESISTANCE_DIV*(voltage/(THERMISTOR_VREF-voltage));  //Calculate Reisistance of Thermistor using Voltage Divider
	float steinhart = resistance / THERMISTOR_R25; //Start Steinhart Temperature Equation
		steinhart = logf(steinhart);
	    steinhart /= THERMISTOR_BETA;
	    steinhart += (1.0f / THERMISTOR_T25_K);
	float temp_k = 1.0f / steinhart;               // Invert to Kelvin
	    return temp_k - 273.15f;                   // Convert Kelvin to Celsius
	}

void Thermistor_Update(Thermistor_t *dev, uint32_t raw_adc) {
    if (dev == 0) return; // NULL Check

    dev->raw_adc = raw_adc; //Update raw_adc
    dev->temperature_c = Thermistor_ConvertRawToTempC(raw_adc); // Find Current Temperature
}
