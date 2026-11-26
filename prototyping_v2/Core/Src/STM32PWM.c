# include "STM32PWM.h"

uint8_t Servo_Init_Custom(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel, uint16_t min_us, uint16_t max_us, uint16_t max_angle) {
    if (servo == NULL || htim == NULL) {
        return 0;
    }
    (servo->htim) = htim;
    (servo->channel) = channel;
    (servo->min_us) = min_us;
    (servo->max_us) = max_us;
    (servo->max_angle) = max_angle;
    if (HAL_TIM_PWM_Start(htim, channel) != HAL_OK) {
        return 0;
    }
    servo->initialized = 1;
    return 1;
}

uint8_t Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel) {
    return Servo_Init_Custom(servo, htim, channel, 500, 2500, 180);
}

void Servo_SetAngle(const Servo_t *servo, float angle) {
    if ((servo == NULL) || (servo->initialized == 0)) {
        return;
    }
    if (angle < 0) {
        angle = 0;
    }
    if (angle > servo->max_angle) {
        angle = servo->max_angle;
    }
    uint16_t pulse_width = servo->min_us + (uint16_t)(((float)angle / servo->max_angle) * (servo->max_us - servo->min_us));
     __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_width);
}

void Servo_SetMicroseconds(const Servo_t *servo, uint16_t microseconds) {
    if ((servo == NULL) || (servo->initialized == 0)) {
        return;
    }
    if (microseconds < servo->min_us) {
        microseconds = servo->min_us;
    }
    if (microseconds > servo->max_us) {
        microseconds = servo->max_us;
    }
     __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, microseconds);
}

// This function accepts 100 as 100% and 0 as 0%. It will convert the percentage to a pulse width in microseconds and set the timer output accordingly.
void Servo_SetPercent(const Servo_t *servo, float percent) {
    if ((servo == NULL) || (servo->initialized == 0)) {
        return;
    }
    if (percent < 0) {
        percent = 0;
    }
    if (percent > 100) {
        percent = 100;
    }
    uint16_t pulse_width = servo->min_us + (uint16_t)((percent / 100) * (servo->max_us - servo->min_us));
     __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_width);
}
