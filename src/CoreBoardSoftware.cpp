#include <Arduino.h>

#include "CoreBoardSoftware.h"

void setup() {
    //Initialize debug serial port
    Serial.begin(9600);
    Serial.println("CoreBoard Setup");

    //Attach Servos to Pins
    leftDriveServo.attach(SERVO_6, 500, 2500);
    leftPanServo.attach(SERVO_5, 500, 2500);
    leftTiltServo.attach(SERVO_4, 500, 2500);

    rightDriveServo.attach(SERVO_3, 500, 2500);
    rightPanServo.attach(SERVO_2, 500, 2500);
    rightTiltServo.attach(SERVO_1, 500, 2500);

    backDriveServo.attach(SERVO_7, 500, 2500);
    
    servo1.attach(SERVO_9, 500, 2500);
    servo2.attach(SERVO_8, 500, 2500);

    
    //Initialize VESC serial ports
    FL_SERIAL.begin(115200);
    ML_SERIAL.begin(115200);
    BL_SERIAL.begin(115200);
    FR_SERIAL.begin(115200);
    MR_SERIAL.begin(115200);
    BR_SERIAL.begin(115200);
    while(!(FL_SERIAL) || !(ML_SERIAL) || !(BL_SERIAL) || !(FR_SERIAL) || !(MR_SERIAL) || !(BR_SERIAL));

    //Configure VESC motors
    FL_Motor.configRampRate(2000);
    ML_Motor.configRampRate(2000);
    BL_Motor.configRampRate(2000);
    FR_Motor.configRampRate(2000);
    MR_Motor.configRampRate(2000);
    BR_Motor.configRampRate(2000);

    // TODO: configure max outputs for Autonomy

    // Initialize Buttons
    pinMode(REVERSE, INPUT);
    pinMode(B_ENC_0, INPUT);
    pinMode(B_ENC_1, INPUT);
    pinMode(B_ENC_2, INPUT);
    pinMode(B_ENC_3, INPUT);

    //Initialize Switches
    pinMode(FL_SWITCH, INPUT_PULLDOWN);
    pinMode(ML_SWITCH, INPUT_PULLDOWN);
    pinMode(BL_SWITCH, INPUT_PULLDOWN);
    pinMode(FR_SWITCH, INPUT_PULLDOWN);
    pinMode(MR_SWITCH, INPUT_PULLDOWN);
    pinMode(BR_SWITCH, INPUT_PULLDOWN);

    //Initialize NeoPixel
    neoPixel.begin();
    neoPixel.setBrightness(MAX_BRIGHTNESS / 2);

    //Start RoveComm
    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_COREBOARD_IPADDRESS);
    Serial.println("Complete.");

    servoStartups();
    feedWatchdog();

    // accelerometer pin got drilled out :(
    // accelerometer.begin();
}

void loop() {
    RoveComm.read(packet);
    
    //Multimedia Packets
    switch(packet.dataId) {
        
        //[R, G, B] -> [(0 - 255), (0 - 255), (0 - 255)]
        case RC_COREBOARD_LEDRGB_DATA_ID:
        {
            uint8_t* data = (uint8_t*)packet.data;
            neoPixel.fill(neoPixel.Color(data[0], data[1], data[2]));
            neoPixel.show();
            break;
        }

        //Flash Pattern selected by data
        case RC_COREBOARD_LEDPATTERNS_DATA_ID:
        {
            uint8_t* data = (uint8_t*)packet.data;
            switch(data[0])
            {
                default:
                    break;
            }

        }

        //[Teleop, Autonomy, Reached Goal] -> Color
        case RC_COREBOARD_STATEDISPLAY_DATA_ID:
        {
            uint8_t* data = (uint8_t*)packet.data;
            switch (data[0])
            {
                case TELEOP:
                    neoPixel.fill(neoPixel.Color(0, 0, 255));
                    neoPixel.show();
                    break;
                
                case AUTONOMY:
                    neoPixel.fill(neoPixel.Color(255, 0, 0));
                    neoPixel.show();
                    break;

                case REACHED_GOAL:
                    for(uint8_t i = 0; i < 5; i++)
                    {
                        neoPixel.fill(neoPixel.Color(0, 255, 0));
                        neoPixel.show();
                        delay(500);
                        neoPixel.clear();
                        neoPixel.show();
                        delay(500);
                    }
                    break;
            }
            break;
        }

        //Set Brightness (0, 255)
        case RC_COREBOARD_BRIGHTNESS_DATA_ID:
        {
            uint8_t* data = (uint8_t*)packet.data;
            if(data[0] >= MAX_BRIGHTNESS) data[0] = MAX_BRIGHTNESS;
            neoPixel.setBrightness(data[0]);
            neoPixel.show();
            break;
        }

    }

    //Gimbal Packets
    switch (packet.dataId) {

        // Increment left drive gimbal by [-180, 180]
        case RC_COREBOARD_LEFTDRIVEGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            leftDriveServo.target += data[0];
            break;

        }

        // Increment right drive gimbal by [-180, 180]
        case RC_COREBOARD_RIGHTDRIVEGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            rightDriveServo.target += data[0];
            break;
        }

        // Increment back drive gimbal by [-180, 180]
        case RC_COREBOARD_BACKDRIVEGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            backDriveServo.target += data[0];
            break;
        }

        // Increment left pan and tilt gimbals by [-180, 180]
        case RC_COREBOARD_LEFTMAINGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            leftPanServo.target += data[0];
            leftTiltServo.target += data[1];
            break;
        }

        // Increment right pan and tilt gimbals by [-180, 180]
        case RC_COREBOARD_RIGHTMAINGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            
            rightPanServo.target += data[0];
            rightTiltServo.target += data[1];
            break;
        }

    }

    //Drive Packets
    switch(packet.dataId) {
        
        //Set All Left and All Right Motors to a DutyCycle [-1, 1]
        case RC_COREBOARD_DRIVELEFTRIGHT_DATA_ID:
        {
            float* data;
            data = (float*)packet.data;

            float leftSpeed = data[0];
            float rightSpeed = data[1];

            for(int i = 0; i < 6; i++) 
                motorTargets[i] = (i < 3) ? leftSpeed : rightSpeed;

            feedWatchdog();
            break;
        }

        //Set All individual Motors to a DutyCycle [-1, 1]
        case RC_COREBOARD_DRIVEINDIVIDUAL_DATA_ID:
        {
            float* data;
            data = (float*)packet.data;

            for(int i = 0; i < 6; i++) 
                motorTargets[i] = data[i];

            feedWatchdog();
            break;
        }

        case RC_COREBOARD_SETWATCHDOGMODE_DATA_ID:
        {
            uint8_t* data = (uint8_t*) packet.data;

            watchdogMode = data[0];
            break;
        }

    }

    uint32_t now = millis();
    
    if (lastDriveUpdate - now >= DRIVE_UPDATE_PERIOD) {
        manualButtons();

        // because drive() also does speed ramping, we can't do caching like for the servos
        // convert to decipercent so RoveVESC can convert BACK to a float
        FL_Motor.drive((int16_t)(motorTargets[0] * 1000));
        ML_Motor.drive((int16_t)(motorTargets[1] * 1000));
        BL_Motor.drive((int16_t)(motorTargets[2] * 1000));
        FR_Motor.drive((int16_t)(motorTargets[3] * 1000));
        MR_Motor.drive((int16_t)(motorTargets[4] * 1000));
        BR_Motor.drive((int16_t)(motorTargets[5] * 1000));

        leftDriveServo.write();
        leftPanServo.write();
        leftTiltServo.write();
        rightDriveServo.write();
        rightPanServo.write();
        rightTiltServo.write();
        backDriveServo.write();
        servo1.write();
        servo2.write();

        lastDriveUpdate = now;
    }

    if (lastTelemetry - now >= TELEMETRY_PERIOD) {
        telemetry();
        lastTelemetry = now;
    }
}

void manualButtons() {
    bool reverse = !digitalRead(REVERSE); // switch is backwards
    uint8_t manualButtons = (digitalRead(B_ENC_3)<<3) | (digitalRead(B_ENC_2)<<2) | (digitalRead(B_ENC_1)<<1) | (digitalRead(B_ENC_0)<<0);
    //  Serial.println(manualButtons);

    // Servos
    switch(manualButtons)
    {
        case DS_EN_BUTTON:  //Motors

            if (digitalRead(FL_SWITCH)) motorTargets[0] = (reverse? -0.5 : 0.5);
            else motorTargets[0] = 0;
    
            if (digitalRead(ML_SWITCH)) motorTargets[1] = (reverse? -0.5 : 0.5);
            else motorTargets[1] = 0;

            if (digitalRead(BL_SWITCH)) motorTargets[2] = (reverse? -0.5 : 0.5);
            else motorTargets[2] = 0;

            if (digitalRead(FR_SWITCH)) motorTargets[3] = (reverse? -0.5 : 0.5);
            else motorTargets[3] = 0;

            if (digitalRead(MR_SWITCH)) motorTargets[4] = (reverse? -0.5 : 0.5);
            else motorTargets[4] = 0;

            if (digitalRead(BR_SWITCH)) motorTargets[5] = (reverse? -0.5 : 0.5);
            else motorTargets[5] = 0;
            break;

        case LD_BUTTON:     //S1
            //  Serial.println("eeeeee");
            leftDriveServo.target += (reverse? -1 : 1);
            break;

        case LP_BUTTON:     //S2
            leftPanServo.target += (reverse? -1 : 1);
            break;

        case LT_BUTTON:     //S3
            leftTiltServo.target += (reverse? -1 : 1);
            break;

        case RD_BUTTON:     //S4
            rightDriveServo.target += (reverse? -1 : 1);
            break;

        case RP_BUTTON:     //S5
            rightPanServo.target += (reverse? -1 : 1);
            break;

        case RT_BUTTON:     //S6
            rightTiltServo.target += (reverse? -1 : 1);
            break;

        case BD_BUTTON:     //S7
            backDriveServo.target += (reverse? -1 : 1);
            break;

        case S1_BUTTON:     //S8
            servo1.target += (reverse? -1 : 1);
            break;

        case S2_BUTTON:     //S9
            servo2.target += (reverse? -1 : 1);
            break;
    }
}

void telemetry() {
    // accelerometer.read();
    // RoveComm.write(RC_COREBOARD_ACCELEROMETERDATA_DATA_ID, RC_COREBOARD_ACCELEROMETERDATA_DATA_COUNT, accelerometer.acceleration);
}

void servoStartups() {
    leftDriveServo.write(LEFT_DRIVE_MIN);
    leftPanServo.write(LEFT_PAN_MIN);
    leftTiltServo.write(LEFT_TILT_MIN);
    rightDriveServo.write(RIGHT_DRIVE_MAX);
    rightPanServo.write(RIGHT_PAN_MAX);
    rightTiltServo.write(RIGHT_TILT_MAX);
    backDriveServo.write(BACK_DRIVE_MIN);

    delay(2000);

    leftDriveServo.write(LEFT_DRIVE_MAX);
    leftPanServo.write(LEFT_PAN_MAX);
    leftTiltServo.write(LEFT_TILT_MAX);
    rightDriveServo.write(RIGHT_DRIVE_MIN);
    rightPanServo.write(RIGHT_PAN_MIN);
    rightTiltServo.write(RIGHT_TILT_MIN);
    backDriveServo.write(BACK_DRIVE_MAX);

    delay(2000);
    
    // the below is necessary even tho we send these during every loop and i have no idea why
    leftDriveServo.write(20);
    leftPanServo.write(90);
    leftTiltServo.write(40);
    rightDriveServo.write(160);
    rightPanServo.write(90);
    rightTiltServo.write(140);
    backDriveServo.write(20);

    delay(50);
}

void estop() {   
    if(!watchdogOverride) {
        for(int i = 0; i < 6; i++) {
            motorTargets[i] = 0;
        }
    }   
}

void feedWatchdog() {
    watchdog.begin(estop, (watchdogMode? WATCHDOG_TIMEOUT_AUTONOMY : WATCHDOG_TIMEOUT_TELEOP));
}
