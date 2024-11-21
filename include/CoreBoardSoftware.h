#ifndef COREBOARD_SOFTWARE_H
#define COREBOARD_SOFTWARE_H

#include "PinAssignments.h"
#include "CachedServo.h"
#include "Accelerometer.h"

#include <RoveComm.h>
#include <RoveVESC.h>

#include <Adafruit_NeoPixel.h>

#define MAX_BRIGHTNESS          70
#define LED_COUNT               256

#define DRIVE_MIN_RPM           2000
#define DRIVE_MAX_RPM           30000
#define DRIVE_MAX_RAMP          0.005

#define LEFT_DRIVE_MIN          0
#define LEFT_PAN_MIN            0
#define LEFT_TILT_MIN           0
#define RIGHT_DRIVE_MIN         40
#define RIGHT_PAN_MIN           0
#define RIGHT_TILT_MIN          0
#define BACK_DRIVE_MIN          0

#define LEFT_DRIVE_MAX          140
#define LEFT_PAN_MAX            180
#define LEFT_TILT_MAX           180
#define RIGHT_DRIVE_MAX         180
#define RIGHT_PAN_MAX           180
#define RIGHT_TILT_MAX          180
#define BACK_DRIVE_MAX          140

#define TELEMETRY_PERIOD        500 // ms
uint32_t lastTelemetry = 0;
void telemetry();

#define WATCHDOG_TIMEOUT_TELEOP         300000
#define WATCHDOG_TIMEOUT_AUTONOMY       1500000
IntervalTimer watchdog;
bool watchdogOverride = false;
uint8_t watchdogMode = 0; // 0: Teleop, 1: Autonomy
void feedWatchdog();

//Rovecomm Declaration
EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);
RoveCommEthernet RoveComm;
RoveCommPacket packet;

Adafruit_NeoPixel neoPixel(LED_COUNT, NEOPIXEL);

#define DRIVE_UPDATE_PERIOD       15 // ms
uint32_t lastDriveUpdate = 0;

//Vesc Serial Declaration
RoveVESC FL_Motor(&FL_SERIAL);
RoveVESC ML_Motor(&ML_SERIAL);
RoveVESC BL_Motor(&BL_SERIAL);
RoveVESC FR_Motor(&FR_SERIAL);
RoveVESC MR_Motor(&MR_SERIAL);
RoveVESC BR_Motor(&BR_SERIAL);

//All wheels are in order of FL, ML, BL, FR, MR, BR
float motorTargets[6] = {0, 0, 0, 0, 0, 0}; // -1.0 to 1.0
int16_t motorCurrent[6] = {0, 0, 0, 0, 0, 0};

//Servo Declarations - Three 9-pin Connectors each with Three Servos
CachedServo leftDriveServo(90, LEFT_DRIVE_MIN, LEFT_DRIVE_MAX);
CachedServo leftPanServo(90, LEFT_PAN_MIN, LEFT_PAN_MAX);
CachedServo leftTiltServo(90, LEFT_TILT_MIN, LEFT_TILT_MAX);

CachedServo rightDriveServo(90, RIGHT_DRIVE_MIN, RIGHT_DRIVE_MAX);
CachedServo rightPanServo(90, RIGHT_PAN_MIN, RIGHT_PAN_MAX);
CachedServo rightTiltServo(90, RIGHT_TILT_MIN, RIGHT_TILT_MAX);

CachedServo backDriveServo(90, BACK_DRIVE_MIN, BACK_DRIVE_MAX);

CachedServo servo1(90, 10, 160), servo2(90, 10, 160);

//Buttons Declaration
uint8_t lastManualButtons = 0;

// Accelerometer
Accelerometer accelerometer(ACC_SDA, ACC_SCL);

// Methods
void estop();
void servoStartups();
void manualButtons();

#endif