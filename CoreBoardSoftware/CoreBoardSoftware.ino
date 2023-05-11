#include "CoreBoardSoftware.h"

void setup() {
    // Initialize debug serial port
    Serial.begin(9600);
    Serial.println("CoreBoard Setup");

    //Attach Servos to Pins
    leftDriveServo.attach(SERVO_1, 500, 2500);
    leftPanServo.attach(SERVO_2, 500, 2500);
    leftTiltServo.attach(SERVO_3, 500, 2500);
    rightDriveServo.attach(SERVO_4, 500, 2500);
    rightPanServo.attach(SERVO_5, 500, 2500);
    rightTiltServo.attach(SERVO_6, 500, 2500);
    servo7.attach(SERVO_7, 500, 2500);
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
    watchdog.begin(estop, WATCHDOG_TIME);
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
            if(leftDriveTarget > SERVO_1_MAX) leftDriveTarget = SERVO_1_MAX;
            if(leftDriveTarget < SERVO_1_MIN) leftDriveTarget = SERVO_1_MIN;
            break;

        }

        // Increment right drive gimbal by [-180, 180]
        case RC_COREBOARD_RIGHTDRIVEGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            rightDriveTarget += data[0];
            if(rightDriveTarget > SERVO_4_MAX) rightDriveTarget = SERVO_4_MAX;
            if(rightDriveTarget < SERVO_4_MIN) rightDriveTarget = SERVO_4_MIN;
            break;
        }

        // Increment left pan and tilt gimbals by [-180, 180]
        case RC_COREBOARD_LEFTMAINGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            leftPanTarget += data[0];
            if(leftPanTarget > SERVO_2_MAX) leftPanTarget = SERVO_2_MAX;
            if(leftPanTarget < SERVO_2_MIN) leftPanTarget = SERVO_2_MIN;
            leftTiltTarget += data[1];
            if(leftTiltTarget > SERVO_3_MAX) leftTiltTarget = SERVO_3_MAX;
            if(leftTiltTarget < SERVO_3_MIN) leftTiltTarget = SERVO_3_MIN;
            break;
        }

        // Increment right pan and tilt gimbals by [-180, 180]
        case RC_COREBOARD_RIGHTMAINGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            rightPanTarget += data[0];
            if(rightPanTarget > SERVO_5_MAX) rightPanTarget = SERVO_5_MAX;
            if(rightPanTarget < SERVO_5_MIN) rightPanTarget = SERVO_5_MIN;
            rightTiltTarget += data[1];
            if(rightTiltTarget > SERVO_6_MAX) rightTiltTarget = SERVO_6_MAX;
            if(rightTiltTarget < SERVO_6_MIN) rightTiltTarget = SERVO_6_MIN;
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

            watchdog.begin(estop, WATCHDOG_TIME);
            break;
        }

        //Set All individual Motors to a DutyCycle [-1, 1]
        case RC_COREBOARD_DRIVEINDIVIDUAL_DATA_ID:
        {
            float* data;
            data = (float*)packet.data;

            for(int i = 0; i < 6; i++) 
                motorTargets[i] = data[i];

            watchdog.begin(estop, WATCHDOG_TIME);
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
    servo7.write(servoTarget7);
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
            leftDriveTarget += (reverse? -3 : 3);
            if(leftDriveTarget > 160) leftDriveTarget = 160;
            if(leftDriveTarget < 10) leftDriveTarget = 10;
            delay(15);
            break;
        
        case 8: //S2
            leftPanTarget += (reverse? -3 : 3);
            if(leftPanTarget > SERVO_2_MAX) leftPanTarget = SERVO_2_MAX;
            if(leftPanTarget < SERVO_2_MIN) leftPanTarget = SERVO_2_MIN;
            delay(15);
            break;
        
        case 9: //S3
            leftTiltTarget += (reverse? -3 : 3);
            if(leftTiltTarget > SERVO_3_MAX) leftTiltTarget = SERVO_3_MAX;
            if(leftTiltTarget < SERVO_3_MIN) leftTiltTarget = SERVO_3_MIN;
            delay(15);
            break;
        
        case 10: //S4
            rightDriveTarget += (reverse? -3 : 3);
            if(rightDriveTarget > 160) rightDriveTarget = 160;
            if(rightDriveTarget < 10) rightDriveTarget = 10;
            delay(15);
            break;
        
        case 11: //S5
            rightPanTarget += (reverse? -3 : 3);
            if(rightPanTarget > 160) rightPanTarget = 160;
            if(rightPanTarget < 10) rightPanTarget = 10;
            delay(15);
            break;
        
        case 12: //S6
            rightTiltTarget += (reverse? -3 : 3);
            if(rightTiltTarget > 160) rightTiltTarget = 160;
            if(rightTiltTarget < 10) rightTiltTarget = 10;
            delay(15);
            break;
        
        case 13: //S7
            servoTarget7 += (reverse? -3 : 3);
            if(servoTarget7 > 160) servoTarget7 = 160;
            if(servoTarget7 < 10) servoTarget7 = 10;
            delay(15);
            break;
        
        case 14: //S8
            servoTarget8 += (reverse? -3 : 3);
            if(servoTarget8 > 160) servoTarget8 = 160;
            if(servoTarget8 < 10) servoTarget8 = 10;
            delay(15);
            break;
        
        case 15: //S9
            servoTarget9 += (reverse? -3 : 3);
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
    leftDriveServo.write(SERVO_1_MIN);
    leftPanServo.write(SERVO_2_MIN);
    leftTiltServo.write(SERVO_3_MIN);
    rightDriveServo.write(SERVO_4_MIN);
    rightPanServo.write(SERVO_5_MIN);
    rightTiltServo.write(SERVO_6_MIN);
    servo7.write(SERVO_7_MIN);
    servo8.write(SERVO_8_MIN);
    servo9.write(SERVO_9_MIN);

    delay(2000);

    leftDriveServo.write(SERVO_1_MAX);
    leftPanServo.write(SERVO_2_MAX);
    leftTiltServo.write(SERVO_3_MAX);
    rightDriveServo.write(SERVO_4_MAX);
    rightPanServo.write(SERVO_5_MAX);
    rightTiltServo.write(SERVO_6_MAX);
    servo7.write(SERVO_7_MAX);
    servo8.write(SERVO_8_MAX);
    servo9.write(SERVO_9_MAX);

    delay(2000);
    
    // the below is necessary even tho we send these during every loop and i have no idea why
    leftDriveServo.write(leftDriveTarget);
    leftPanServo.write(leftPanTarget);
    leftTiltServo.write(leftTiltTarget);
    rightDriveServo.write(rightDriveTarget);
    rightPanServo.write(rightPanTarget);
    rightTiltServo.write(rightTiltTarget);
    servo7.write(servoTarget7);
    servo8.write(servoTarget8);
    servo9.write(servoTarget9);

    delay(50);
}

void estop() 
{    
    if(!watchdogOverride) 
    {
        for(int i = 0; i < 6; i++) 
            motorTargets[i] = 0;

        watchdog.begin(estop, WATCHDOG_TIME);
    }   
}
