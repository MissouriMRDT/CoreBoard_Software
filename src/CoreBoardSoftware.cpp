#include "CoreBoardSoftware.h"
#include <Arduino.h>

void setup() {
    // wheels

    // choose whether to reverse wheels
    FL_Motor.configInvert(false);
    ML_Motor.configInvert(false);
    BL_Motor.configInvert(false);
    FR_Motor.configInvert(false);
    MR_Motor.configInvert(false);
    BR_Motor.configInvert(false);

    // initialize VESC serial ports
    FL_SERIAL.begin(115200);
    ML_SERIAL.begin(115200);
    BL_SERIAL.begin(115200);
    FR_SERIAL.begin(115200);
    MR_SERIAL.begin(115200);
    BR_SERIAL.begin(115200);
    // wait for all serial ports to connect
    while(!(FL_SERIAL) || !(ML_SERIAL) || !(BL_SERIAL) || !(FR_SERIAL) || !(MR_SERIAL) || !(BR_SERIAL));

    // initialize Drive Mode
    driveMode(true);

    // servos
    Spare1.attach(SPARE_1_SERVO);
    Spare2.attach(SPARE_2_SERVO);
    LeftPan.attach(LEFT_PAN_SERVO);
    LeftTilt.attach(LEFT_TILT_SERVO);
    BackPan.attach(BACK_PAN_SERVO);
    BackTilt.attach(BACK_TILT_SERVO);
    RightPan.attach(RIGHT_PAN_SERVO);
    RightTilt.attach(RIGHT_TILT_SERVO);

    // rotary encoder
    pinMode(RTRY_1, INPUT_PULLDOWN);
    pinMode(RTRY_2, INPUT_PULLDOWN);
    pinMode(RTRY_4, INPUT_PULLDOWN);

    // D-pad buttons
    pinMode(DIR_FORWARD, INPUT_PULLUP);
    pinMode(DIR_BACK, INPUT_PULLUP);
    pinMode(DIR_RIGHT, INPUT_PULLUP);
    pinMode(DIR_LEFT, INPUT_PULLUP);

    // turn on fans
    pinMode(FAN_PWM_1, OUTPUT);
    analogWrite(FAN_PWM_1, 255);

    accelerometer.begin();
    // TODO: set up temperature IC

    // initialize rovecomm
    RoveComm.begin(RC_COREBOARD_IPADDRESS);

    backPanel.begin();

    nextTelemetry = millis();
    nextDriveUpdate = millis();

    feedWatchdog();
}

void loop() {
    handleButtons();

    // responding to commands
    RoveCommPacket packet;
    RoveComm.read(packet);
    switch (packet.dataId) {
        case RC_COREBOARD_DRIVELEFTRIGHT_DATA_ID:
            for (int i = 0; i < 3; i++) {
                wheelSpeeds[i] = packet.fdata[0];
                wheelSpeeds[i + 3] = packet.fdata[1];
            }
            feedWatchdog();
            break;
        case RC_COREBOARD_DRIVEINDIVIDUAL_DATA_ID:
            for (int i = 0; i < 6; i++) {
                wheelSpeeds[i] = packet.fdata[i];
            }
            feedWatchdog();
            break;
        case RC_COREBOARD_LEFTGIMBAL_DATA_ID:
            LeftPan.write(packet.i16data[0]);
            LeftTilt.write(packet.i16data[1]);
            break;
        case RC_COREBOARD_RIGHTGIMBAL_DATA_ID:
            RightPan.write(packet.i16data[0]);
            RightTilt.write(packet.i16data[1]);
            break;
        case RC_COREBOARD_BACKGIMBAL_DATA_ID:
            BackPan.write(packet.i16data[0]);
            BackTilt.write(packet.i16data[1]);
            break;
        case RC_COREBOARD_WATCHDOGOVERRIDE_DATA_ID:
            watchdogOverride = packet.u8data[0];
            break;
    }

    if (millis() >= nextTelemetry) {
        telemetry();
        nextTelemetry += TELEMETRY_PERIOD;
    }

    if (millis() >= nextDriveUpdate) {
        for (int i = 0; i < 6; i++) {
            motors[i]->drive((int)(wheelSpeeds[i] * 1000));
        }
        nextDriveUpdate += TELEMETRY_PERIOD;
    }
}

void handleButtons() {
    int mode = digitalRead(RTRY_1) | (digitalRead(RTRY_2) << 1) | (digitalRead(RTRY_4) << 2);
    switch (mode) {
        case 0:
            break;
        case 1:
            driveMast(LeftPan, LeftTilt);
            break;
        case 2:
            driveMast(Spare1, Spare1);
            break;
        case 3:
            driveMast(BackPan, BackTilt);
            break;
        case 4:
            driveMast(Spare2, Spare2);
            break;
        case 5:
            driveMast(RightPan, RightTilt);
            break;
        case 6:
        {
            const int switches[] = {FL_SWITCH, ML_SWITCH, BL_SWITCH, FR_SWITCH, MR_SWITCH, BR_SWITCH};
            for (int i = 0; i < 6; i++) {
                if (digitalRead(switches[i])) {
                    if (!digitalRead(DIR_FORWARD)) {
                        wheelSpeeds[i] = 0.5f;
                        feedWatchdog();
                    } else if (!digitalRead(DIR_BACK)) {
                        wheelSpeeds[i] = -0.5f;
                        feedWatchdog();
                    } else {
                        wheelSpeeds[i] = 0.0f;
                    }
                }
            }
            break;
        }
        case 7:
            break;
    }
}

void driveMast(PWMServo& pan, PWMServo& tilt) {
    if (!digitalRead(DIR_FORWARD)) {
        tilt.write(tilt.read() + 1);
    }
    if (!digitalRead(DIR_BACK)) {
        tilt.write(tilt.read() - 1);
    }
    if (!digitalRead(DIR_RIGHT)) {
        pan.write(pan.read() + 1);
    }
    if (!digitalRead(DIR_LEFT)) {
        pan.write(pan.read() - 1);
    }
}

void driveMode(bool isTeleop) {
    if (isTeleop) {
        for (int i = 0; i < 6; i++) {
            motors[i]->configMaxOutputs(-TELEOP_MAX_SPEED, TELEOP_MAX_SPEED);
            motors[i]->configRampRate(TELEOP_MAX_RAMP_RATE);
        }
    } else {
        for (int i = 0; i < 6; i++) {
            motors[i]->configMaxOutputs(-AUTONOMY_MAX_SPEED, AUTONOMY_MAX_SPEED);
            motors[i]->configRampRate(AUTONOMY_MAX_RAMP_RATE);
        }
    }
}

void telemetry() {
    RoveComm.write(RC_COREBOARD_ACCELEROMETERDATA_DATA_ID, 3, accelerometer.acceleration);
    // TODO: measure temperature and fan speeds
    // float motorSpeeds[6];
    float motorCurrents[6];
    float vescCurrents[6];
    for (int i = 0; i < 6; i++) {
        VescValues values = motors[i]->getVescTelemetry();
        // motorSpeeds[i] = values.rpm / 14; // divide by motor poles
        motorCurrents[i] = values.avgMotorCurrent;
        vescCurrents[i] = values.avgInputCurrent;
        if (values.error != FAULT_CODE_NONE) {
            uint8_t errorData[2] = {(uint8_t)i, values.error};
            RoveComm.write(RC_COREBOARD_VESCFAULT_DATA_ID, 2, errorData);
        }
    }
    // RoveComm.write(RC_COREBOARD_MOTORSPEEDS_DATA_ID, RC_COREBOARD_MOTORSPEEDS_DATA_COUNT, motorSpeeds);
    RoveComm.write(RC_COREBOARD_MOTORCURRENTS_DATA_ID, RC_COREBOARD_MOTORCURRENTS_DATA_COUNT, motorCurrents);
    RoveComm.write(RC_COREBOARD_VESCCURRENTS_DATA_ID, RC_COREBOARD_VESCCURRENTS_DATA_COUNT, vescCurrents);
}

void setDisplayState(DisplayState newState) {
    displayState = newState;
    lightingPanelChanged = true;
    displayStateProgress = 0;
}

void updateLightingPanel() {
    switch (displayState) {
        case DisplayState::OFF:
            backPanel.clear();
            break;
        case DisplayState::TELEOP:
            backPanel.fill(0x0000FF); // Blue
            break;
        case DisplayState::AUTONOMY:
            backPanel.fill(0xFF0000); // Red
            break;
        case DisplayState::REACHED_GOAL:
        {
            uint32_t lastColor = backPanel.getPixelColor(0);
            uint32_t nextColor = (displayStateProgress / 1000) % 2 == 0 ? 0x00FF00 : 0x000000; // Blink green each second
            if (lastColor != nextColor) {
                lightingPanelChanged = true;
            }
            backPanel.fill(nextColor);
            break;
        }
        case DisplayState::CUSTOM:
            backPanel.fill(customDisplayColor);
            break;
    }
    if (lightingPanelChanged) {
        backPanel.show(); // this takes like 7ms so we want to call it as little as possible.
        lightingPanelChanged = false;
    }

    displayStateProgress += LIGHTING_PANEL_UPDATE_PERIOD;
}

void estop() {
    if(!watchdogOverride) {
        for(int i = 0; i < 6; i++) {
            wheelSpeeds[i] = 0;
        }
    }
}

void feedWatchdog() {
    watchdog.begin(estop, (watchdogMode? WATCHDOG_TIMEOUT_AUTONOMY : WATCHDOG_TIMEOUT_TELEOP));
}
