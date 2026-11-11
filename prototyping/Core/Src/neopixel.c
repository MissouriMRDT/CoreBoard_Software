/*
 * neopixel.c
 *
 *  Created on: Nov 6, 2026
 *      Author: luker
 */

#include "neopixel.h"
#include <string.h>

static Pixel_t pixel_array[NUM_LEDS];
static uint32_t pwm_dma_buffer[PWM_BUF_SIZE];

void Neopixel_Init(void){ 							// Clears the RAM space for the Pixel Array and DMA buffer
	memset(pixel_array, 0, sizeof(pixel_array));
	memset(pwm_dma_buffer, 0 , sizeof(pwm_dma_buffer));
}

void Neopixel_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= NUM_LEDS) return;
    pixel_array[index].r = r;
    pixel_array[index].g = g;
    pixel_array[index].b = b;
}

void Neopixel_SetAll(uint8_t r, uint8_t g, uint8_t b) {
    for (uint16_t i = 0; i < NUM_LEDS; i++) {
        Neopixel_SetPixel(i, r, g, b);
    }
}

void Neopixel_Render(TIM_HandleTypeDef *htim, uint32_t Channel){
	uint32_t buf_idx = 0;

	for (uint16_t i = 0; i < NUM_LEDS; i++ ){ 					// Iterates through every LED in the LED strip

		uint32_t color   =	((uint8_t)pixel_array[i].g) << 16|	// Compiles a 32 bit variable with 8 bits unused and rest being color Bytes
							((uint8_t)pixel_array[i].r) << 8|
							((uint8_t)pixel_array[i].b);

		for (int8_t bit = 23; bit >= 0; bit--){ 				// Iterates through every bit in Color ignoring the unsused 8 bits and compiles the full DMA buffer
			if ((color >> bit) & 1){
				pwm_dma_buffer[buf_idx++] = PWM_BIT_1;
			} else{
				pwm_dma_buffer[buf_idx++] = PWM_BIT_0;
			}
		}
	}

	while (buf_idx < PWM_BUF_SIZE) { 							// Gaurentees the 50 Tick Delay at the end of the Data
	        pwm_dma_buffer[buf_idx++] = 0;
	    }

	HAL_TIM_PWM_Start_DMA(htim, Channel, pwm_dma_buffer, PWM_BUF_SIZE);  // Non Blocking DMA Data sending
}


