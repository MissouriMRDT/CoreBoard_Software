#include "CoreBoardSoftware.h"

void setup() {
    // Initialize debug serial port
    Serial.begin(9600);
    Serial.println("CoreBoard Setup");

    //Attach Servos to Pins
    leftDriveServo.attach(SERVO_4, 500, 2500);
    leftPanServo.attach(SERVO_5, 500, 2500);
    leftTiltServo.attach(SERVO_6, 500, 2500);
    rightDriveServo.attach(SERVO_1, 500, 2500);
    rightPanServo.attach(SERVO_2, 500, 2500);
    rightTiltServo.attach(SERVO_3, 500, 2500);
    backDriveServo.attach(SERVO_7, 500, 2500);
    servo8.attach(SERVO_8, 500, 2500);
    servo9.attach(SERVO_9, 500, 2500);

    
    //Initialize VESC serial ports
    FL_SERIAL.begin(115200);
    ML_SERIAL.begin(115200);
    BL_SERIAL.begin(115200);
    FR_SERIAL.begin(115200);
    MR_SERIAL.begin(115200);
    BR_SERIAL.begin(115200);
    while(!(FL_SERIAL) || !(ML_SERIAL) || !(BL_SERIAL) || !(FR_SERIAL) || !(MR_SERIAL) || !(BR_SERIAL));

    FL_Motor.setSerialPort(&FL_SERIAL);
    ML_Motor.setSerialPort(&ML_SERIAL);
    BL_Motor.setSerialPort(&BL_SERIAL);
    FR_Motor.setSerialPort(&FR_SERIAL);
    MR_Motor.setSerialPort(&MR_SERIAL);
    BR_Motor.setSerialPort(&BR_SERIAL);

    //Initialize Buttons
    pinMode(REVERSE, INPUT);
    pinMode(B_ENC_0, INPUT);
    pinMode(B_ENC_1, INPUT);
    pinMode(B_ENC_2, INPUT);
    pinMode(B_ENC_3, INPUT);

    //Initialize LED
    pinMode(LED_TX, OUTPUT);
    pinMode(LED_RX, OUTPUT);
    digitalWrite(LED_TX, LOW);
    digitalWrite(LED_RX, LOW);

    //Initialize NeoPixel
    neoPixel.begin();
    neoPixel.setBrightness(MAX_BRIGHTNESS);


    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_COREBOARD_FIRSTOCTET, RC_COREBOARD_SECONDOCTET, RC_COREBOARD_THIRDOCTET, RC_COREBOARD_FOURTHOCTET, &TCPServer);
    Serial.println("Complete.");

    servoStartups();
    feedWatchdog();
    lastTimestamp = millis();
}

void loop() 
{
    uint32_t timestamp = millis();
    packet = RoveComm.read();

    
    //Multimedia Packets
    switch(packet.data_id) {
        
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

                default:
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
    switch (packet.data_id) {

        // Increment left drive gimbal by [-180, 180]
        case RC_COREBOARD_LEFTDRIVEGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            leftDriveTarget += data[0];
            if(leftDriveTarget > LEFT_DRIVE_MAX) leftDriveTarget = LEFT_DRIVE_MAX;
            if(leftDriveTarget < LEFT_DRIVE_MIN) leftDriveTarget = LEFT_DRIVE_MIN;
            break;

        }

        // Increment right drive gimbal by [-180, 180]
        case RC_COREBOARD_RIGHTDRIVEGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            rightDriveTarget += data[0];
            if(rightDriveTarget > RIGHT_DRIVE_MAX) rightDriveTarget = RIGHT_DRIVE_MAX;
            if(rightDriveTarget < RIGHT_DRIVE_MIN) rightDriveTarget = RIGHT_DRIVE_MIN;
            break;
        }

        // Increment back drive gimbal by [-180, 180]
        case RC_COREBOARD_BACKDRIVEGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            backDriveTarget += data[0];
            if(backDriveTarget > BACK_DRIVE_MAX) backDriveTarget = BACK_DRIVE_MAX;
            if(backDriveTarget < BACK_DRIVE_MIN) backDriveTarget = BACK_DRIVE_MIN;
            break;
        }

        // Increment left pan and tilt gimbals by [-180, 180]
        case RC_COREBOARD_LEFTMAINGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            leftPanTarget += data[0];
            if(leftPanTarget > LEFT_PAN_MAX) leftPanTarget = LEFT_PAN_MAX;
            if(leftPanTarget < LEFT_PAN_MIN) leftPanTarget = LEFT_PAN_MIN;
            leftTiltTarget += data[1];
            if(leftTiltTarget > LEFT_TILT_MAX) leftTiltTarget = LEFT_TILT_MAX;
            if(leftTiltTarget < LEFT_TILT_MIN) leftTiltTarget = LEFT_TILT_MIN;
            break;
        }

        // Increment right pan and tilt gimbals by [-180, 180]
        case RC_COREBOARD_RIGHTMAINGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            rightPanTarget += data[0];
            if(rightPanTarget > RIGHT_PAN_MAX) rightPanTarget = RIGHT_PAN_MAX;
            if(rightPanTarget < RIGHT_PAN_MIN) rightPanTarget = RIGHT_PAN_MIN;
            rightTiltTarget += data[1];
            if(rightTiltTarget > RIGHT_TILT_MAX) rightTiltTarget = RIGHT_TILT_MAX;
            if(rightTiltTarget < RIGHT_TILT_MIN) rightTiltTarget = RIGHT_TILT_MIN;
            break;
        }

    }
    
    

    //Drive Packets
    switch(packet.data_id) {
        
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


    manualButtons();


    // Ramp
    float ramp = (timestamp - lastTimestamp) * DRIVE_MAX_RAMP;
    for(int i = 0; i < 6; i++) {
        if((motorTargets[i] > motorSpeeds[i]) && ((motorTargets[i] - motorSpeeds[i]) > ramp)) {
            motorSpeeds[i] += ramp;
        }
        else if((motorTargets[i] < motorSpeeds[i]) && ((motorTargets[i] - motorSpeeds[i]) < -ramp)) {
            motorSpeeds[i] -= ramp;
        }
        else {
            motorSpeeds[i] = motorTargets[i];
        }
    }


    // Outputs
    FL_Motor.setDuty(motorSpeeds[0]);
    ML_Motor.setDuty(motorSpeeds[1]);
    BL_Motor.setDuty(motorSpeeds[2]);
    FR_Motor.setDuty(motorSpeeds[3]);
    MR_Motor.setDuty(motorSpeeds[4]);
    BR_Motor.setDuty(motorSpeeds[5]);

    leftDriveServo.write(leftDriveTarget);
    leftPanServo.write(leftPanTarget);
    leftTiltServo.write(leftTiltTarget);
    rightDriveServo.write(rightDriveTarget);
    rightPanServo.write(rightPanTarget);
    rightTiltServo.write(rightTiltTarget);
    backDriveServo.write(backDriveTarget);
    servo8.write(servoTarget8);
    servo9.write(servoTarget9);


    lastTimestamp = timestamp;
}


void manualButtons()
{
    bool reverse = digitalRead(REVERSE);
    uint8_t manualButtons = (digitalRead(B_ENC_3)<<3) | (digitalRead(B_ENC_2)<<2) | (digitalRead(B_ENC_1)<<1) | (digitalRead(B_ENC_0)<<0);

    // FL
    if (manualButtons == 3) motorSpeeds[0] = (reverse? -0.5 : 0.5);
    else if (lastManualButtons == 3) motorSpeeds[0] = 0;
    
    // ML
    if (manualButtons == 5) motorSpeeds[1] = (reverse? -0.5 : 0.5);
    else if (lastManualButtons == 5) motorSpeeds[1] = 0;
    
    // BL
    if (manualButtons == 6) motorSpeeds[2] = (reverse? -0.5 : 0.5);
    else if (lastManualButtons == 6) motorSpeeds[2] = 0;

    // FR
    if (manualButtons == 1) motorSpeeds[3] = (reverse? -0.5 : 0.5);
    else if (lastManualButtons == 1) motorSpeeds[3] = 0;

    // MR
    if (manualButtons == 2) motorSpeeds[4] = (reverse? -0.5 : 0.5);
    else if (lastManualButtons == 2) motorSpeeds[4] = 0;

    // BR
    if (manualButtons == 4) motorSpeeds[5] = (reverse? -0.5 : 0.5);
    else if (lastManualButtons == 4) motorSpeeds[5] = 0;


    // Servos
    switch(manualButtons)
    {
        case 7: //S1
            leftDriveTarget += (reverse? -1 : 1);
            if(leftDriveTarget > LEFT_DRIVE_MAX) leftDriveTarget = LEFT_DRIVE_MAX;
            if(leftDriveTarget < LEFT_DRIVE_MIN) leftDriveTarget = LEFT_DRIVE_MIN;
            delay(15);
            break;
        
        case 8: //S2
            leftPanTarget += (reverse? -1 : 1);
            if(leftPanTarget > LEFT_PAN_MAX) leftPanTarget = LEFT_PAN_MAX;
            if(leftPanTarget < LEFT_PAN_MIN) leftPanTarget = LEFT_PAN_MIN;
            delay(15);
            break;
        
        case 9: //S3
            leftTiltTarget += (reverse? -1 : 1);
            if(leftTiltTarget > LEFT_TILT_MAX) leftTiltTarget = LEFT_TILT_MAX;
            if(leftTiltTarget < LEFT_TILT_MIN) leftTiltTarget = LEFT_TILT_MIN;
            delay(15);
            break;
        
        case 10: //S4
            rightDriveTarget += (reverse? -1 : 1);
            if(rightDriveTarget > RIGHT_DRIVE_MAX) rightDriveTarget = RIGHT_DRIVE_MAX;
            if(rightDriveTarget < RIGHT_DRIVE_MIN) rightDriveTarget = RIGHT_DRIVE_MIN;
            delay(15);
            break;
        
        case 11: //S5
            rightPanTarget += (reverse? -1 : 1);
            if(rightPanTarget > RIGHT_PAN_MAX) rightPanTarget = RIGHT_PAN_MAX;
            if(rightPanTarget < RIGHT_PAN_MIN) rightPanTarget = RIGHT_PAN_MIN;
            delay(15);
            break;
        
        case 12: //S6
            rightTiltTarget += (reverse? -1 : 1);
            if(rightTiltTarget > RIGHT_TILT_MAX) rightTiltTarget = RIGHT_TILT_MAX;
            if(rightTiltTarget < RIGHT_TILT_MIN) rightTiltTarget = RIGHT_TILT_MIN;
            delay(15);
            break;
        
        case 13: //S7
            backDriveTarget += (reverse? -1 : 1);
            if(backDriveTarget > BACK_DRIVE_MAX) backDriveTarget = BACK_DRIVE_MAX;
            if(backDriveTarget < BACK_DRIVE_MIN) backDriveTarget = BACK_DRIVE_MIN;
            delay(15);
            break;
        
        case 14: //S8
            servoTarget8 += (reverse? -1 : 1);
            if(servoTarget8 > 160) servoTarget8 = 160;
            if(servoTarget8 < 10) servoTarget8 = 10;
            delay(15);
            break;
        
        case 15: //S9
            servoTarget9 += (reverse? -1 : 1);
            if(servoTarget9 > 160) servoTarget9 = 160;
            if(servoTarget9 < 10) servoTarget9 = 10;
            delay(15);
            break;

        default:
            break;
    }

    
    lastManualButtons = manualButtons;
}

void servoStartups()
{
    leftDriveServo.write(LEFT_DRIVE_MIN);
    leftPanServo.write(LEFT_PAN_MIN);
    leftTiltServo.write(LEFT_TILT_MIN);
    rightDriveServo.write(RIGHT_DRIVE_MIN);
    rightPanServo.write(RIGHT_PAN_MIN);
    rightTiltServo.write(RIGHT_TILT_MIN);
    backDriveServo.write(BACK_DRIVE_MIN);

    delay(2000);

    leftDriveServo.write(LEFT_DRIVE_MAX);
    leftPanServo.write(LEFT_PAN_MAX);
    leftTiltServo.write(LEFT_TILT_MAX);
    rightDriveServo.write(RIGHT_DRIVE_MAX);
    rightPanServo.write(RIGHT_PAN_MAX);
    rightTiltServo.write(RIGHT_TILT_MAX);
    backDriveServo.write(BACK_DRIVE_MAX);

    delay(2000);
    
    // the below is necessary even tho we send these during every loop and i have no idea why
    leftDriveServo.write(leftDriveTarget);
    leftPanServo.write(leftPanTarget);
    leftTiltServo.write(leftTiltTarget);
    rightDriveServo.write(rightDriveTarget);
    rightPanServo.write(rightPanTarget);
    rightTiltServo.write(rightTiltTarget);
    backDriveServo.write(backDriveTarget);

    delay(50);
}

void estop() 
{    
    if(!watchdogOverride) {
        for(int i = 0; i < 6; i++) {
            motorTargets[i] = 0;
        }
    }   
}

void feedWatchdog() {
    watchdog.begin(estop, (watchdogMode? WATCHDOG_TIMEOUT_AUTONOMY : WATCHDOG_TIMEOUT_TELEOP));
}