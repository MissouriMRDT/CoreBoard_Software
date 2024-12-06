#ifndef PIN_ASSIGNMENTS_H
#define PIN_ASSIGNMENTS_H

//Drive Pins
#define FL_TX           29
#define FL_RX           28
#define ML_TX           24
#define ML_RX           25
#define BL_TX           35
#define BL_RX           34

#define FR_TX           20
#define FR_RX           21
#define MR_TX           17
#define MR_RX           16
#define BR_TX           14
#define BR_RX           15

#define FL_SERIAL       Serial7
#define ML_SERIAL       Serial6
#define BL_SERIAL       Serial8
#define FR_SERIAL       Serial5
#define MR_SERIAL       Serial4
#define BR_SERIAL       Serial3

//Servo Pins
#define SERVO_1         0
#define SERVO_2         1
#define SERVO_3         2
#define SERVO_4         3
#define SERVO_5         4
#define SERVO_6         5
#define SERVO_7         6
#define SERVO_8         7
#define SERVO_9         8

//NeoPixels Pins
#define NEOPIXEL        11

//Button Pins
#define B_ENC_0         23
#define B_ENC_1         22
#define B_ENC_2         41
#define B_ENC_3         33

#define REVERSE         10

//Switch Pins
#define FL_Switch       38
#define ML_Switch       27
#define BL_Switch       26
#define FR_Switch       12
#define MR_Switch       40
#define BR_Switch       39

//Accelerometer Pins
#define ACC_SDA         18
#define ACC_SCL         19

// Button Assignments (Switch Number -> Encoder Number)
// Motors
#define DS_EN_BUTTON    8

#define FL_BUTTON       14
#define ML_BUTTON       13
#define BL_BUTTON       12
#define FR_BUTTON       15
#define MR_BUTTON       1
#define BR_BUTTON       2
// Servos
#define LT_BUTTON       3
#define LP_BUTTON       2
#define LD_BUTTON       1
#define RT_BUTTON       7
#define RP_BUTTON       10
#define RD_BUTTON       9
#define BD_BUTTON       6
#define S1_BUTTON       4
#define S2_BUTTON       5

#endif