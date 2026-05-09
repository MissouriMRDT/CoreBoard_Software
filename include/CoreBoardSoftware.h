#ifndef COREBOARD_SOFTWARE_H
#define COREBOARD_SOFTWARE_H

#include "Accelerometer.h"
#include "PinAssignments.h"

#include <RoveComm.h>
#include <RoveVESC.h>
#include "RoveServo.h"

#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP9808.h>
#include <Bounce.h>

RoveCommEthernet RoveComm;

// Buttons
Bounce buttonForward(DIR_FORWARD, 10);
Bounce buttonBack(DIR_BACK, 10);
Bounce buttonLeft(DIR_LEFT, 10);
Bounce buttonRight(DIR_RIGHT, 10);


// Wheels
#define TELEOP_MAX_SPEED        1000
#define TELEOP_MAX_RAMP_RATE    1200
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
RoveServo Spare1;
RoveServo Spare2;
RoveServo LeftPan;
RoveServo LeftTilt;
RoveServo BackPan;
RoveServo BackTilt;
RoveServo RightPan;
RoveServo RightTilt;

void handleButtons();
void driveMast(RoveServo& pan, RoveServo& tilt);

// Telemetry
#define TELEMETRY_PERIOD 750 // in milliseconds
uint32_t nextTelemetry = 0;
void telemetry();
Accelerometer accelerometer(SENS_SDA, SENS_SCL);
Adafruit_MCP9808 tempSensor;
uint32_t tachometerPulses = 0;

// Lighting panel stuff
#define MAX_BRIGHTNESS          70
#define LED_COUNT               64

Adafruit_NeoPixel backPanel(64, BACK_STRIP_PIN);
Adafruit_NeoPixel innerStrip(64, INNER_STRIP_PIN);

//////////// TEMPORARY ////////////

// Current lighting panel is not a NeoPixel, so separate pins switch red, green, and blue lines
uint8_t RGBStripBrightness = MAX_BRIGHTNESS;
uint32_t RGBStripColor = 0x000000;
void setRGBStripColor(uint32_t rgb);
void setRGBStripBrightness(uint8_t brightness);

//////////// TEMPORARY ////////////

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

// Watchdog
#define WATCHDOG_TIMEOUT_TELEOP         1000000
#define WATCHDOG_TIMEOUT_AUTONOMY       1500000
IntervalTimer watchdog;
bool watchdogOverride = false;
DriveMode watchdogMode = DriveMode::TELEOP; // 0: Teleop, 1: Autonomy
void feedWatchdog();

#endif
