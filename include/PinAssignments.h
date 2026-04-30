#ifndef PIN_ASSIGNMENTS_H
#define PIN_ASSIGNMENTS_H

//Drive Pins
#define FL_TX           35
#define FL_RX           34
#define ML_TX           14
#define ML_RX           15
#define BL_TX           20
#define BL_RX           21

#define FR_TX           29
#define FR_RX           28
#define MR_TX           24
#define MR_RX           25
#define BR_TX           1
#define BR_RX           0

#define FL_SERIAL       Serial8
#define ML_SERIAL       Serial3
#define BL_SERIAL       Serial5
#define FR_SERIAL       Serial7
#define MR_SERIAL       Serial6
#define BR_SERIAL       Serial1

//Servo Pins
#define LEFT_TILT_SERVO       8
#define LEFT_PAN_SERVO        7
#define RIGHT_TILT_SERVO      6
#define RIGHT_PAN_SERVO       2
#define BACK_TILT_SERVO       11
#define BACK_PAN_SERVO        12
#define SPARE_1_SERVO         22
#define SPARE_2_SERVO         5

//Button Pins
#define RTRY_1          26
#define RTRY_2          30
#define RTRY_4          31

#define DIR_FORWARD     32
#define DIR_BACK        13
#define DIR_RIGHT       23
#define DIR_LEFT        27

//Motor Switches
#define FL_SWITCH       36
#define ML_SWITCH       37
#define BL_SWITCH       38
#define FR_SWITCH       39
#define MR_SWITCH       40
#define BR_SWITCH       41

//Sensor I2C Pins
#define SENS_SDA         18
#define SENS_SCL         19

//Fan Pins
#define FAN_PWM_1        10
#define FAN_TACH_1       3

//NeoPixels Pins
#define NEOPIXEL_1_PIN    17
#define NEOPIXEL_2_PIN    16
//RGB Strip Pins
#define RED_PIN         4
#define GREEN_PIN       33
#define BLUE_PIN        9

#endif
