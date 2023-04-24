#ifndef CoreBoardSoftware_h
#define CoreBoardSoftware_h

#include "PinAssignments.h"

#include <RoveComm.h>
#include <RoveCommManifest.h>
#include <RoveCommPacket.h>
#include <VescUart.h>

#include <Servo.h>
#include <Adafruit_NeoPixel.h>


#define MAX_BRIGHTNESS          130
#define LED_COUNT               256

#define DRIVE_MIN_RPM           2000
#define DRIVE_MAX_RPM           30000
#define DRIVE_MAX_RAMP          0.005

#define TELEMETRY_UPDATE        150000
IntervalTimer telemetry;

#define WATCHDOG_TIME           300000
IntervalTimer watchdog;
bool watchdogOverride = false;


//Rovecomm Declaration
EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);
RoveCommEthernet RoveComm;
rovecomm_packet packet;

uint32_t lastUpdateTime;
uint32_t lastRampTime;
uint32_t maxRamp;

Adafruit_NeoPixel neoPixel = Adafruit_NeoPixel(LED_COUNT, NEOPIXEL);

//Vesc Serial Declaration
VescUart FL_Motor;
VescUart FR_Motor;
VescUart ML_Motor;
VescUart MR_Motor;
VescUart BL_Motor;
VescUart BR_Motor;

//All wheels are in order of FL, ML, BL, FR, MR, BR
float motorTargets[6] = {0, 0, 0, 0, 0, 0}; //FL, ML, BL, FR, MR, BR
float motorSpeeds[6] = {0, 0, 0, 0, 0, 0}; //FL, ML, BL, FR, MR, BR

//Servo Declarations - Three 9-pin Connectors each with Three Servos
Servo leftPanServo, leftTiltServo, leftDriveServo;
Servo rightPanServo, rightTiltServo, rightDriveServo;
Servo servo7, servo8, servo9;

//Buttons Declaration
uint8_t lastManualButtons = 0;

//Estop Declaration
void EStop();
void Telemetry();

#endif