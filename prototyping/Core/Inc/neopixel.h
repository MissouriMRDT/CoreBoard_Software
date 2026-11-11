/*
 * neopixel.h
 *
 *  Created on: Nov 6, 2026
 *      Author: luker
 */

#ifndef INC_NEOPIXEL_H_
#define INC_NEOPIXEL_H_

#include "main.h"

#define NUM_LEDS 10 		// Dummy Value, replace with actual LED count later
#define RESET_SLOTS 50 		// Neopixels require 50 uS buffer to change LED, this assures a 62.5 uS pause
#define PWM_BUF_SIZE ((NUM_LEDS * 24) + RESET_SLOTS) // Full Length of Array for 1 Full LED Frame

#define PWM_BIT_0 11 		// Clock Ticks to Pulse a 0 bit
#define PWM_BIT_1 22		// Clock Ticks to Puple a 1 bit

typedef struct{
	uint32_t r;
	uint32_t g;
	uint32_t b;
} Pixel_t;

void Neopixel_Init(void);													// Neopixel Initialization
void Neopixel_SetPixel(uint16_t index, uint8_t r, uint8_t g,uint8_t b);		// Sets an individual pixel to a given color
void Neopixel_SetAll(uint8_t r, uint8_t g,uint8_t b);						// Sets all pixels to a given color
void Neopixel_Render(TIM_HandleTypeDef *htim, uint32_t Channel);			// Translator of Binary Colors to Buffered Neopixel-ready data

#endif /* INC_NEOPIXEL_H_ */
