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
    setDriveMode(DriveMode::TELEOP);

    // servos (LDX-227)
    Spare1.attach(SPARE_1_SERVO, 500, 2500);
    Spare2.attach(SPARE_2_SERVO, 500, 2500);
    LeftPan.attach(LEFT_PAN_SERVO, 500, 2500);
    LeftTilt.attach(LEFT_TILT_SERVO, 500, 2500);
    BackPan.attach(BACK_PAN_SERVO, 500, 2500);
    BackTilt.attach(BACK_TILT_SERVO, 500, 2500);
    RightPan.attach(RIGHT_PAN_SERVO, 500, 2500);
    RightTilt.attach(RIGHT_TILT_SERVO, 500, 2500);

    Spare1.configAngleRange(0, 270);
    Spare2.configAngleRange(0, 270);
    LeftPan.configAngleRange(0, 270);
    LeftTilt.configAngleRange(0, 270);
    BackPan.configAngleRange(0, 270);
    BackTilt.configAngleRange(0, 270);
    RightPan.configAngleRange(0, 270);
    RightTilt.configAngleRange(0, 270);

    // Spare1.configSoftLimits(0, 270);
    // Spare2.configSoftLimits(0, 270);
    LeftPan.configSoftLimits(0, 270);
    LeftTilt.configSoftLimits(30, 210);
    BackPan.configSoftLimits(50, 270);
    BackTilt.configSoftLimits(10, 200);
    RightPan.configSoftLimits(0, 270);
    RightTilt.configSoftLimits(0, 180);

    Spare1.write(90);
    Spare2.write(90);
    LeftPan.write(90);
    LeftTilt.write(120);
    BackPan.write(120);
    BackTilt.write(80);
    RightPan.write(220);
    RightTilt.write(90);

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
    analogWrite(FAN_PWM_1, 127);

    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    analogWriteFrequency(RED_PIN, 400);
    analogWriteFrequency(GREEN_PIN, 400);
    analogWriteFrequency(BLUE_PIN, 400);

    accelerometer.begin();

    // initialize rovecomm
    RoveComm.begin(RC_COREBOARD_IPADDRESS);

    NeoPixel1.begin();
    NeoPixel1.setBrightness(MAX_BRIGHTNESS);
    NeoPixel1.fill(0xFF0000);
    NeoPixel1.show();

    NeoPixel2.begin();
    NeoPixel2.setBrightness(MAX_BRIGHTNESS);
    NeoPixel2.fill(0xFF0000);
    NeoPixel2.show();

    setRGBStripBrightness(255); // The strip isn't as bright as the old panels
    setDisplayState(DisplayState::OFF);

    nextTelemetry = millis();

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
            driveWheels();
            feedWatchdog();
            break;
        case RC_COREBOARD_DRIVEINDIVIDUAL_DATA_ID:
            for (int i = 0; i < 6; i++) {
                wheelSpeeds[i] = packet.fdata[i];
            }
            driveWheels();
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
        case RC_COREBOARD_SETWATCHDOGMODE_DATA_ID:
            setDriveMode((DriveMode)packet.u8data[0]);
            break;
        case RC_COREBOARD_STATEDISPLAY_DATA_ID:
            switch ((COREBOARD_DISPLAYSTATE)packet.u8data[0]) {
                case TELEOP:
                    setDisplayState(DisplayState::TELEOP);
                    break;
                case AUTONOMY:
                    setDisplayState(DisplayState::AUTONOMY);
                    break;
                case REACHED_GOAL:
                    setDisplayState(DisplayState::REACHED_GOAL);
                    break;
            }
            break;
        case RC_COREBOARD_BRIGHTNESS_DATA_ID:
            NeoPixel1.setBrightness(packet.u8data[0]);
            NeoPixel2.setBrightness(packet.u8data[0]);
            setRGBStripBrightness(packet.u8data[0]);
            break;
        case RC_COREBOARD_LEDRGB_DATA_ID:
            customDisplayColor = Adafruit_NeoPixel::Color(packet.u8data[0], packet.u8data[1], packet.u8data[2]);
            setDisplayState(DisplayState::CUSTOM);
            break;
        case RC_COREBOARD_INTERNALRGB_DATA_ID:
            NeoPixel1.fill(Adafruit_NeoPixel::Color(packet.u8data[0], packet.u8data[1], packet.u8data[2]));
            NeoPixel1.show();
            NeoPixel2.fill(Adafruit_NeoPixel::Color(packet.u8data[0], packet.u8data[1], packet.u8data[2]));
            NeoPixel2.show();
            break;
    }

    if (millis() >= nextTelemetry) {
        telemetry();
        nextTelemetry += TELEMETRY_PERIOD;
    }

    updateLightingPanel();

    delay(10);
}

void handleButtons() {
    buttonForward.update();
    buttonBack.update();
    buttonLeft.update();
    buttonRight.update();

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
            // active low
            bool forward = !buttonForward.read();
            bool back = !buttonBack.read();
            bool left = !buttonLeft.read();
            bool right = !buttonRight.read();

            bool shouldDrive = false;
            if (forward && !back) {
                for (int i = 0; i < 6; i++) {
                    wheelSpeeds[i] = digitalRead(switches[i]) ? 0.5f : 0;
                }
                shouldDrive = true;
            } else if (back && !forward) {
                for (int i = 0; i < 6; i++) {
                    wheelSpeeds[i] = digitalRead(switches[i]) ? -0.5f : 0;
                }
                shouldDrive = true;
            } else if (left && !right) {
                for (int i = 0; i < 3; i++) {
                    wheelSpeeds[i] = digitalRead(switches[i]) ? -0.5f : 0;
                    wheelSpeeds[i + 3] = digitalRead(switches[i + 3]) ? 0.5f : 0;
                }
                shouldDrive = true;
            } else if (right && !left) {
                for (int i = 0; i < 3; i++) {
                    wheelSpeeds[i] = digitalRead(switches[i]) ? 0.5f : 0;
                    wheelSpeeds[i + 3] = digitalRead(switches[i + 3]) ? -0.5f : 0;
                }
                shouldDrive = true;
            } else if (
                // Otherwise stop if any buttons have triggered or untriggered
                buttonForward.risingEdge() ||
                buttonBack.risingEdge() ||
                buttonLeft.risingEdge() ||
                buttonRight.risingEdge() ||
                buttonForward.fallingEdge() ||
                buttonBack.fallingEdge() ||
                buttonLeft.fallingEdge() ||
                buttonRight.fallingEdge()
            ) {
                for (int i = 0; i < 6; i++) {
                    wheelSpeeds[i] = 0;
                }
                shouldDrive = true;
            }
            if (shouldDrive) {
                driveWheels();
            }
            break;
        }
        case 7:
            if (buttonRight.fallingEdge()) {
                if (displayState != DisplayState::CUSTOM) {
                    displayState = (DisplayState)((4 + (int)displayState + 1) % 4);
                } else {
                    displayState = DisplayState::OFF;
                }

            } else if (buttonLeft.fallingEdge()) {
                if (displayState != DisplayState::CUSTOM) {
                    displayState = (DisplayState)((4 + (int)displayState - 1) % 4);
                } else {
                    displayState = DisplayState::OFF;
                }
            }
            break;
    }
}

void driveMast(RoveServo& pan, RoveServo& tilt) {
    if (!buttonForward.read()) {
        tilt.write(tilt.read() + 1);
    }
    if (!buttonBack.read()) {
        tilt.write(tilt.read() - 1);
    }
    if (!buttonLeft.read()) {
        pan.write(pan.read() - 1);
    }
    if (!buttonRight.read()) {
        pan.write(pan.read() + 1);
    }
}

void driveWheels() {
    for (int i = 0; i < 6; i++) {
        motors[i]->drive((int)(wheelSpeeds[i] * 1000));
    }
}

void setDriveMode(DriveMode mode) {
    switch (mode) {
        case DriveMode::TELEOP:
            for (int i = 0; i < 6; i++) {
                motors[i]->configMaxOutputs(-TELEOP_MAX_SPEED, TELEOP_MAX_SPEED);
                motors[i]->configRampRate(TELEOP_MAX_RAMP_RATE);
            }
            break;
        case DriveMode::AUTONOMY:
            for (int i = 0; i < 6; i++) {
                motors[i]->configMaxOutputs(-AUTONOMY_MAX_SPEED, AUTONOMY_MAX_SPEED);
                motors[i]->configRampRate(AUTONOMY_MAX_RAMP_RATE);
            }
            break;
    }
    watchdogMode = mode;
}

void telemetry() {
    accelerometer.read();
    RoveComm.write(RC_COREBOARD_ACCELEROMETERDATA_DATA_ID, 3, accelerometer.acceleration);
    // TODO: measure temperature and fan speeds
    float motorSpeeds[6];
    float motorCurrents[6];
    float vescCurrents[6];
    for (int i = 0; i < 6; i++) {
        VescValues values = motors[i]->getVescTelemetry();
        motorSpeeds[i] = values.rpm / 14; // divide by motor poles
        motorCurrents[i] = values.avgMotorCurrent;
        vescCurrents[i] = values.avgInputCurrent;
        if (values.error != FAULT_CODE_NONE) {
            uint8_t errorData[2] = {(uint8_t)i, values.error};
            RoveComm.write(RC_COREBOARD_VESCFAULT_DATA_ID, 2, errorData);
        }
    }
    RoveComm.write(RC_COREBOARD_MOTORSPEEDS_DATA_ID, RC_COREBOARD_MOTORSPEEDS_DATA_COUNT, motorSpeeds);
    RoveComm.write(RC_COREBOARD_MOTORCURRENTS_DATA_ID, RC_COREBOARD_MOTORCURRENTS_DATA_COUNT, motorCurrents);
    RoveComm.write(RC_COREBOARD_VESCCURRENTS_DATA_ID, RC_COREBOARD_VESCCURRENTS_DATA_COUNT, vescCurrents);
}

//////////// TEMPORARY ////////////

void setRGBStripColor(uint32_t rgb) {
    if (RGBStripColor != rgb) {
        lightingPanelChanged = true;
        RGBStripColor = rgb;
    }
    analogWrite(RED_PIN, (((rgb >> 16) & 0xFF) * RGBStripBrightness) >> 8);
    analogWrite(GREEN_PIN, (((rgb >> 8) & 0xFF) * RGBStripBrightness) >> 8);
    analogWrite(BLUE_PIN, ((rgb & 0xFF) * RGBStripBrightness) >> 8);
}

void setRGBStripBrightness(uint8_t brightness) {
    RGBStripBrightness = brightness;
    setRGBStripColor(RGBStripColor); // update PWM
}

//////////// TEMPORARY ////////////

void setDisplayState(DisplayState newState) {
    displayState = newState;
    lightingPanelChanged = true;
    displayStateProgress = 0;
}

void updateLightingPanel() {
    switch (displayState) {
        case DisplayState::OFF:
            NeoPixel1.clear();
            NeoPixel2.clear();
            setRGBStripColor(0x000000);
            break;
        case DisplayState::TELEOP:
            NeoPixel1.fill(0x0000FF);
            NeoPixel2.fill(0x0000FF);
            setRGBStripColor(0x0000FF);
            break;
        case DisplayState::AUTONOMY:
            NeoPixel1.fill(0xFF0000); // Red
            NeoPixel2.fill(0xFF0000); // Red
            setRGBStripColor(0xFF0000);
            break;
        case DisplayState::REACHED_GOAL:
        {
            uint32_t lastColor = NeoPixel1.getPixelColor(0);
            uint32_t nextColor = (displayStateProgress / 1000) % 2 == 0 ? 0x00FF00 : 0x000000; // Blink green each second
            if (lastColor != nextColor) {
                lightingPanelChanged = true;
            }
            NeoPixel1.fill(nextColor);
            NeoPixel2.fill(nextColor);
            setRGBStripColor(nextColor);
            break;
        }
        case DisplayState::CUSTOM:
            setRGBStripColor(customDisplayColor);
            NeoPixel1.fill(customDisplayColor);
            NeoPixel2.fill(customDisplayColor);
            break;
    }
    if (lightingPanelChanged) {
        NeoPixel1.show();
        NeoPixel2.show(); // this takes like 7ms so we want to call it as little as possible.
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
    driveWheels();
}

void feedWatchdog() {
    switch (watchdogMode) {
        case DriveMode::TELEOP:
            watchdog.begin(estop, WATCHDOG_TIMEOUT_TELEOP);
            break;
        case DriveMode::AUTONOMY:
            watchdog.begin(estop, WATCHDOG_TIMEOUT_AUTONOMY);
            break;
    }
}
