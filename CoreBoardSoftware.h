#ifndef CoreBoardSoftware_h
#define CoreBoardSoftware_h

#include <RoveComm.h>
#include <RoveCommManifest.h>
#include <RoveCommPacket.h>
#include <Servo.h>
#include <VescUart.h>
#include "Adafruit_NeoPixel.h"

#define NUM_SERVOS 7

#define RD_SERVO        0
#define RMP_SERVO       1
#define RMT_SERVO       2
#define LD_SERVO        3
#define LMP_SERVO       4
#define LMT_SERVO       5
#define SPARE1_SERVO    6

#define RD_SERVO_PIN    1
#define RMP_SERVO_PIN   2
#define RMT_SERVO_PIN   3
#define LD_SERVO_PIN    4
#define LMP_SERVO_PIN   5
#define LMT_SERVO_PIN   6
#define SPARE1_PIN      33

Servo servos[NUM_SERVOS];
int16_t servoPosition[NUM_SERVOS];
uint16_t servoMax[NUM_SERVOS];
uint16_t servoMin[NUM_SERVOS];

//Starting Positions
#define SERVO1_START 135
#define SERVO2_START 135
#define SERVO3_START 135
#define SERVO4_START 135
#define SERVO5_START 135
#define SERVO6_START 135
#define SERVO7_START 135
#define SERVO8_START 135

//Minimum and maximum servo values
#define RD_SERVO_MIN    0
#define RD_SERVO_MAX    180
#define RMP_SERVO_MIN   0
#define RMP_SERVO_MAX   180
#define RMT_SERVO_MIN   0
#define RMT_SERVO_MAX   180
#define LD_SERVO_MIN    0
#define LD_SERVO_MAX    180
#define LMP_SERVO_MIN   0
#define LMP_SERVO_MAX   180
#define LMT_SERVO_MIN   0
#define LMT_SERVO_MAX   180
#define SPARE1_MIN      0
#define SPARE1_MAX      180

//Buttons
#define RD_BUTTON       24
#define RMP_BUTTON      25
#define RMT_BUTTON      26
#define LD_BUTTON       27
#define LMP_BUTTON      28
#define LMT_BUTTON      29
#define SPARE1_BUTTON   30

#define BUTTON_INC_VAL  10

//Switch
#define RIGHT_SWITCH    11
#define LEFT_SWITCH     12

//Motor Overide Buttons
#define FL_MOTOR                20
#define FR_MOTOR                21
#define ML_MOTOR                11
#define MR_MOTOR                12
#define BL_MOTOR                25
#define BR_MOTOR                24
#define DIR_SWITCH              33
#define BUTTON_OVERIDE_SPEED    0.15

//Motor Speed Controls
#define FL_SERIAL               Serial1
#define FR_SERIAL               Serial2
#define ML_SERIAL               Serial7
#define MR_SERIAL               Serial8
#define BL_SERIAL               Serial4
#define BR_SERIAL               Serial3
#define DRIVE_MIN_RPM           2000
#define DRIVE_MAX_RPM           30000
#define DRIVE_MAX_RAMP          0.005

#define BRIGHTNESS      130
#define AUTONOMY_COUNT  256
#define AUTONOMY_PANEL  11

#define TELEMETRY_UPDATE        150000
#define WATCHDOG_TIME           300000

//Rovecomm
RoveCommEthernet RoveComm;
rovecomm_packet packet;
uint32_t lastUpdateTime;
uint32_t lastRampTime;
uint32_t maxRamp;
bool watchdogOverride = false;
EthernetServer TCPServer(RC_ROVECOMM_COREBOARD_PORT);
IntervalTimer watchdog;
IntervalTimer telemetry;
int16_t *position;
uint8_t* data;

//Vesc Serial
VescUart FL_UART;
VescUart FR_UART;
VescUart ML_UART;
VescUart MR_UART;
VescUart BL_UART;
VescUart BR_UART;

//All wheels are in order of FL, ML, BL, FR, MR, BR
uint8_t motorButtons[6] = {FL_MOTOR, ML_MOTOR, BL_MOTOR, FR_MOTOR, MR_MOTOR, BR_MOTOR};
int16_t motorTargets[6] = {0, 0, 0, 0, 0, 0}; //FL, ML, BL, FR, MR, BR
int16_t motorSpeeds[6] = {0, 0, 0, 0, 0, 0}; //FL, ML, BL, FR, MR, BR
float motorCurrent[6] = {};

Adafruit_NeoPixel autonomy = Adafruit_NeoPixel(AUTONOMY_COUNT, AUTONOMY_PANEL);

//Estop Decleration
void EStop();
void Telemetry();

#endif