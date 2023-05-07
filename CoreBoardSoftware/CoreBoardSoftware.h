#ifndef CoreBoardSoftware_h
#define CoreBoardSoftware_h

#include "PinAssignments.h"

#include <RoveComm.h>
#include <VescUart.h>

#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#define MAX_BRIGHTNESS          70
#define LED_COUNT               256

#define DRIVE_MIN_RPM           2000
#define DRIVE_MAX_RPM           30000
#define DRIVE_MAX_RAMP          0.005

#define SERVO_1_MIN             10    //leftDrive
#define SERVO_2_MIN             50    //leftPan
#define SERVO_3_MIN             20    //leftTilt
#define SERVO_4_MIN             10    //rightDrive
#define SERVO_5_MIN             10    //rightPan
#define SERVO_6_MIN             10    //rightTilt
#define SERVO_7_MIN             10
#define SERVO_8_MIN             10
#define SERVO_9_MIN             10

#define SERVO_1_MAX             160   //leftDrive
#define SERVO_2_MAX             130   //leftPan
#define SERVO_3_MAX             115   //leftTilt
#define SERVO_4_MAX             160   //rightDrive
#define SERVO_5_MAX             160   //rightPan
#define SERVO_6_MAX             160   //rightTilt
#define SERVO_7_MAX             160
#define SERVO_8_MAX             160
#define SERVO_9_MAX             160

#define TELEMETRY_UPDATE        150000
IntervalTimer telemetry;

#define WATCHDOG_TIME           300000
IntervalTimer watchdog;
bool watchdogOverride = false;


//Rovecomm Declaration
EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);
RoveCommEthernet RoveComm;
rovecomm_packet packet;

uint32_t lastTimestamp;

Adafruit_NeoPixel neoPixel = Adafruit_NeoPixel(LED_COUNT, NEOPIXEL);

//Vesc Serial Declaration
VescUart FL_Motor;
VescUart FR_Motor;
VescUart ML_Motor;
VescUart MR_Motor;
VescUart BL_Motor;
VescUart BR_Motor;

//All wheels are in order of FL, ML, BL, FR, MR, BR
float motorTargets[6] = {0, 0, 0, 0, 0, 0};
float motorSpeeds[6] = {0, 0, 0, 0, 0, 0};
int16_t motorCurrent[6] = {0, 0, 0, 0, 0, 0};

//Servo Declarations - Three 9-pin Connectors each with Three Servos
Servo leftDriveServo, leftPanServo, leftTiltServo;
Servo rightDriveServo, rightPanServo, rightTiltServo;
Servo servo7, servo8, servo9;

//All servos are in order of leftDrive, leftPan, leftTilt, rightDrive, rightPan, rightTilt, 7, 8, 9
int16_t leftDriveTarget = 85;
int16_t leftPanTarget = 90;
int16_t leftTiltTarget = 65;
int16_t rightDriveTarget = 85;
int16_t rightPanTarget = 85;
int16_t rightTiltTarget = 85;
int16_t servoTarget7 = 85;
int16_t servoTarget8 = 85;
int16_t servoTarget9 = 85;

//Buttons Declaration
uint8_t lastManualButtons = 0;

// Methods
void estop();
void servoStartups();
void manualButtons();

#endif