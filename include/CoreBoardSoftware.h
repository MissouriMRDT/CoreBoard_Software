#ifndef COREBOARD_SOFTWARE_H
#define COREBOARD_SOFTWARE_H

#include "Accelerometer.h"
#include "PinAssignments.h"

#include <RoveComm.h>
#include <RoveVESC.h>

#include <Adafruit_NeoPixel.h>
#include <PWMServo.h>

RoveCommEthernet RoveComm;

// Wheels

#define TELEOP_MAX_SPEED        1000
#define TELEOP_MAX_RAMP_RATE    500
#define AUTONOMY_MAX_SPEED      1000
#define AUTONOMY_MAX_RAMP_RATE  0 // Disable ramping for Autonomy

//[FL, ML, BL, FR, MR, BR]
float wheelSpeeds[6] = {0};

RoveVESC FL_Motor(&FL_SERIAL);
RoveVESC ML_Motor(&ML_SERIAL);
RoveVESC BL_Motor(&BL_SERIAL);
RoveVESC FR_Motor(&FR_SERIAL);
RoveVESC MR_Motor(&MR_SERIAL);
RoveVESC BR_Motor(&BR_SERIAL);
RoveVESC* motors[6] = {&FL_Motor, &ML_Motor, &BL_Motor, &FR_Motor, &MR_Motor, &BR_Motor};

enum class DriveMode {
    TELEOP = 0,
    AUTONOMY = 1
};
//Drive Mode Set Ramp Rates and Max Speed
void setDriveMode(DriveMode mode);
void driveWheels();

// Servos
PWMServo Spare1;
PWMServo Spare2;
PWMServo LeftPan;
PWMServo LeftTilt;
PWMServo BackPan;
PWMServo BackTilt;
PWMServo RightPan;
PWMServo RightTilt;

void handleButtons();
void driveMast(PWMServo& pan, PWMServo& tilt);

// Telemetry
#define TELEMETRY_PERIOD 750 // in milliseconds
uint32_t nextTelemetry = 0;
void telemetry();
Accelerometer accelerometer(SENS_SDA, SENS_SCL);

// Lighting panel stuff
Adafruit_NeoPixel backPanel(256, BACK_STRIP_PIN);
Adafruit_NeoPixel innerStrip(256, INNER_STRIP_PIN);

// This will eventually be phased out in favor of the RoveLighting library which runs its own state machine
enum class DisplayState {
    OFF, TELEOP, AUTONOMY, REACHED_GOAL, CUSTOM
};
DisplayState displayState = DisplayState::OFF;
uint32_t customDisplayColor = 0x000000; // Adafruit_NeoPixel::Color(r, g, b) -> int
void setDisplayState(DisplayState newState);
uint32_t displayStateProgress = 0;

void updateLightingPanel();
uint32_t lastLightingPanelUpdate = 0;
bool lightingPanelChanged = true;
#define LIGHTING_PANEL_UPDATE_PERIOD 100 // ms

#define MAX_BRIGHTNESS          70
#define LED_COUNT               256

// Watchdog
#define WATCHDOG_TIMEOUT_TELEOP         1000000
#define WATCHDOG_TIMEOUT_AUTONOMY       1500000
IntervalTimer watchdog;
bool watchdogOverride = false;
DriveMode watchdogMode = DriveMode::TELEOP; // 0: Teleop, 1: Autonomy
void feedWatchdog();

#endif
