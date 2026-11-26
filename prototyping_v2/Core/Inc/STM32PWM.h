#ifndef STM32PWM_H
#define STM32PWM_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t min_us;
    uint16_t max_us;
    uint16_t max_angle;
    uint8_t initialized;
} Servo_t;

// Setup functions to initialize the servo object with specific parameters.
// This allows later functions to only need the servo object to function properly.
// NOTE: these function assume you have already initialized the timer output and set the correct prescalar & counter
//       period values within CubeMX. Your prescalar should be configured to give a 1 MHz timer frequency, or 1 us per tick.
//       For most standard servos, the frequency at which the servo is driven is 50Hz, or a period of 20 ms. This means that
//       the timer counter period should be set to 19999 ticks for 20 ms. (this is due to the timer starting at 0)
// NOTE: You MUST initialize the timer output before calling this function, or the timer will not output any signal.
//       This should be done automatically by CubeMX if you have the pin configured properly.
uint8_t Servo_Init_Custom(Servo_t *servo,  TIM_HandleTypeDef *htim, uint32_t channel, uint16_t min_us, uint16_t max_us, uint16_t max_angle);
uint8_t Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);

// Sets the output of the configured timer basen on the given arguments.
// Each of these functions serve the same purpose, but accept different math for ease of use.
void Servo_SetAngle(const Servo_t *servo, float angle);
void Servo_SetMicroseconds(const Servo_t *servo, uint16_t microseconds);
void Servo_SetPercent(const Servo_t *servo, float percent);

#endif
