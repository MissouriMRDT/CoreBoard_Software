#include "CoreBoardSoftware.h"

void setup() {

    // Initialize debug serial port
    Serial.begin(9600);
    Serial.println("Serial init");

    //Attach Servos to Pins
    leftPanServo.attach(SERVO_1);
    leftTiltServo.attach(SERVO_2);
    leftDriveServo.attach(SERVO_3);
    rightPanServo.attach(SERVO_4);
    rightTiltServo.attach(SERVO_5);
    rightDriveServo.attach(SERVO_6);
    servo7.attach(SERVO_7);
    servo8.attach(SERVO_8);
    servo9.attach(SERVO_9);
    
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

    //Initialize NeoPixel
    neoPixel.begin();
    neoPixel.setBrightness(MAX_BRIGHTNESS);
    
    //Start RoveComm
    RoveComm.begin(RC_COREBOARDBOARD_FIRSTOCTET, RC_COREBOARDBOARD_SECONDOCTET, RC_COREBOARDBOARD_THIRDOCTET, RC_COREBOARDBOARD_FOURTHOCTET);

    telemetry.begin(Telemetry, TELEMETRY_UPDATE);
    
    lastRampTime = millis();

    watchdog.begin(EStop, WATCHDOG_TIME);
}

void loop() 
{ 
    //Read incoming packet
    packet = RoveComm.read();

    //Multimedia Packets
    switch(packet.data_id) {
        //[R, G, B] -> [(0 - 255), (0 - 255), (0 - 255)]
        case RC_MULTIMEDIABOARD_LEDRGB_DATA_ID:
        {
            uint8_t* data = (uint8_t*)packet.data;
            neoPixel.fill(neoPixel.Color(data[0], data[1], data[2]));
            break;
        }

        //Flash Pattern selected by data
        case RC_MULTIMEDIABOARD_LEDPATTERNS_DATA_ID:
        {
            uint8_t* data = (uint8_t*)packet.data;
            switch(data[0])
            {
                default:
                    break;
            }

        }

        //[Teleop, Autonomy, Reached Goal] -> Color
        case RC_MULTIMEDIABOARD_STATEDISPLAY_DATA_ID:
        {
            uint8_t* data = (uint8_t*)packet.data;
            switch (data[0])
            {
                case TELEOP:
                    neoPixel.fill(neoPixel.Color(0, 0, 255));
                    break;
                
                case AUTONOMY:
                    neoPixel.fill(neoPixel.Color(255, 0, 0));
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
        case RC_MULTIMEDIABOARD_BRIGHTNESS_DATA_ID:
        {
            uint8_t* data = (uint8_t*)packet.data;
            if(data[0] >= MAX_BRIGHTNESS) data[0] = MAX_BRIGHTNESS;
            neoPixel.setBrightness(data[0]);
            break;
        }

        default:
        {
            break;
        }
    }



    //Gimbal Packets
    switch (packet.data_id) {

        // Increment left drive gimbal by [-180, 180]
        case RC_GIMBALBOARD_LEFTDRIVEGIMBALINCREMENT_DATA_ID:
        {
            int16_t data = ((int16_t*) packet.data)[0];
            incrementGimbal(leftDriveServo, data);
            break;
        }

        // Increment right drive gimbal by [-180, 180]
        case RC_GIMBALBOARD_RIGHTDRIVEGIMBALINCREMENT_DATA_ID:
        {
            int16_t data = ((int16_t*) packet.data)[0];
            incrementGimbal(rightDriveServo, data);
            break;
        }

        // Increment left pan and tilt gimbals by [-180, 180]
        case RC_GIMBALBOARD_LEFTMAINGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            incrementGimbal(leftPanServo, data[0]);
            incrementGimbal(leftTiltServo, data[1]);
            break;
        }

        // Increment right pan and tilt gimbals by [-180, 180]
        case RC_GIMBALBOARD_RIGHTMAINGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            incrementGimbal(rightPanServo, data[0]);
            incrementGimbal(rightTiltServo, data[1]);
            break;
        }

        default:
            break;
    }
    
    

    //Drive Packets
    switch(packet.data_id) {
        
        //Set All Left and All Right Motors to a DutyCycle [-1, 1]
        case RC_DRIVEBOARD_DRIVELEFTRIGHT_DATA_ID:
        {
            float* mutualSpeeds;
            mutualSpeeds = (float*)packet.data;

            float leftSpeed = mutualSpeeds[0];
            float rightSpeed = mutualSpeeds[1];

            for(int i = 0; i < 6; i++) 
                motorTargets[i] = (i < 3) ? leftSpeed : rightSpeed;

            watchdog.begin(EStop, WATCHDOG_TIME);

            FL_Motor.setDuty((float)leftSpeed);
            ML_Motor.setDuty((float)leftSpeed);
            BL_Motor.setDuty((float)leftSpeed);
            FR_Motor.setDuty((float)rightSpeed);
            MR_Motor.setDuty((float)rightSpeed);
            BR_Motor.setDuty((float)rightSpeed);
            break;
        }

        //Set All individual Motors to a DutyCycle [-1, 1]
        case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATA_ID:
        {
            float* speeds;
            speeds = (float*)packet.data;

            for(int i = 0; i < 6; i++) 
                motorTargets[i] = speeds[i];

            watchdog.begin(EStop, WATCHDOG_TIME);
                        
            FL_Motor.setDuty((float)speeds[0]);
            ML_Motor.setDuty((float)speeds[1]);
            BL_Motor.setDuty((float)speeds[2]);
            FR_Motor.setDuty((float)speeds[3]);
            MR_Motor.setDuty((float)speeds[4]);
            BR_Motor.setDuty((float)speeds[5]);
            break;
        }
    }
    
/*
    maxRamp = (millis() - lastRampTime) * DRIVE_MAX_RAMP;
    for(int i = 0; i < 6; i++)
    {
        if((motorTargets[i] > motorSpeeds[i]) && ((motorTargets[i] - motorSpeeds[i]) > maxRamp))
        {
            motorSpeeds[i] += maxRamp;
            Serial.println("Ramping Up");
        }
        else if((motorTargets[i] < motorSpeeds[i]) && ((motorTargets[i] - motorSpeeds[i]) < -maxRamp))
        {
            motorSpeeds[i] -= maxRamp;
            Serial.println("Ramping Down");
        }
        else 
            motorSpeeds[i] = motorTargets[i];
    }

    FL_Motor.setDuty((float)motorSpeeds[0]);
    ML_Motor.setDuty((float)motorSpeeds[1]);
    BL_Motor.setDuty((float)motorSpeeds[2]);
    FR_Motor.setDuty((float)motorSpeeds[3]);
    MR_Motor.setDuty((float)motorSpeeds[4]);
    BR_Motor.setDuty((float)motorSpeeds[5]);
    */

    neoPixel.show();

    lastRampTime = millis();
}

void EStop() 
{    
    if(!watchdogOverride) 
    {
        for(int i = 0; i < 6; i++) 
            motorTargets[i] = 0;

        watchdog.begin(EStop, WATCHDOG_TIME);
    }   
}

void Telemetry()
{
    //Converts Motor Current to a value from {-DRIVE_MAX_RPM, DRIVE_MAX_RPM} -> {-1000, 1000}
    if(FL_Motor.getVescValues()) {
        motorCurrent[0] = (float)map(FL_Motor.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[0] = 0;
    }


    if(ML_Motor.getVescValues()) {
        motorCurrent[1] = (float)map(ML_Motor.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[1] = 0;
    }


    if(BL_Motor.getVescValues()) {
        motorCurrent[2] = (float)map(BL_Motor.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[2] = 0;
    }

    
    if(FR_Motor.getVescValues()) {
        motorCurrent[3] = (float)map(FR_Motor.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[3] = 0;
    }


    if(MR_Motor.getVescValues()) {
        motorCurrent[4] = (float)map(MR_Motor.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[4] = 0;
    }


    if(BR_Motor.getVescValues()) {
        motorCurrent[5] = (float)map(BR_Motor.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[5] = 0;
    }

    RoveComm.write(RC_DRIVEBOARD_DRIVESPEEDS_DATA_ID, RC_DRIVEBOARD_DRIVESPEEDS_DATA_COUNT, motorCurrent);
}


void incrementServo(Servo servo, const int16_t &incrementVal)
{
    int16_t target = servo.read() + incrementVal;
    if(target > 180) target = 180;
    if(target < 0) target = 0;

    servo.write(target);
}

void manualButtons()
{
    bool reverse = digitalRead(REVERSE);
    uint8_t manualButtons = (digitalRead(B_ENC_3)<<3) | (digitalRead(B_ENC_2)<<2) | (digitalRead(B_ENC_1)<<1) | (digitalRead(B_ENC_0)<<0);

    if(manualButtons == lastManualButtons){
        switch(manualButtons){
            case 1: //FR
                motorSpeeds[3] = (reverse? -1 : 1);
                break;
            
            case 2: //FM (MR)
                motorSpeeds[4] = (reverse? -1 : 1);
                break;
            
            case 3: //FL
                motorSpeeds[0] = (reverse? -1 : 1);
                break;
            
            case 4: //BR
                motorSpeeds[5] = (reverse? -1 : 1);
                break;
            
            case 5: //BM (ML)
                motorSpeeds[1] = (reverse? -1 : 1);
                break;
            
            case 6: //BL
                motorSpeeds[2] = (reverse? -1 : 1);
                break;

            case 7: //S1
                leftPanServo.write((reverse? 0 : 180));
                break;
            
            case 8: //S2
                leftTiltServo.write((reverse? 0 : 180));
                break;
            
            case 9: //S3
                leftDriveServo.write((reverse? 0 : 180));
                break;
            
            case 10: //S4
                rightPanServo.write((reverse? 0 : 180));
                break;
            
            case 11: //S5
                rightTiltServo.write((reverse? 0 : 180));
                break;
            
            case 12: //S6
                rightDriveServo.write((reverse? 0 : 180));
                break;
            
            case 13: //S7
                servo7.write((reverse? 0 : 180));
                break;
            
            case 14: //S8
                servo8.write((reverse? 0 : 180));
                break;
            
            case 15: //S9
                servo9.write((reverse? 0 : 180));
                break;

            default:
                break;
        }
    } else {
        switch(lastManualButtons){
            case 1: //FR
                motorSpeeds[3] = 0;
                break;
            
            case 2: //FM (MR)
                motorSpeeds[4] = 0;
                break;
            
            case 3: //FL
                motorSpeeds[0] = 0;
                break;
            
            case 4: //BR
                motorSpeeds[5] = 0;
                break;
            
            case 5: //BM (ML)
                motorSpeeds[1] = 0;
                break;
            
            case 6: //BL
                motorSpeeds[2] = 0;
                break;

            case 7: //S1
                leftPanServo.write(90);
                break;
            
            case 8: //S2
                leftTiltServo.write(90);
                break;
            
            case 9: //S3
                leftDriveServo.write(90);
                break;
            
            case 10: //S4
                rightPanServo.write(90);
                break;
            
            case 11: //S5
                rightTiltServo.write(90);
                break;
            
            case 12: //S6
                rightDriveServo.write(90);
                break;
            
            case 13: //S7
                servo7.write(90);
                break;
            
            case 14: //S8
                servo8.write(90);
                break;
            
            case 15: //S9
                servo9.write(90);
                break;

            default:
                break;
        }
    }

    FL_Motor.setDuty((float)motorSpeeds[0]);
    ML_Motor.setDuty((float)motorSpeeds[1]);
    BL_Motor.setDuty((float)motorSpeeds[2]);
    FR_Motor.setDuty((float)motorSpeeds[3]);
    MR_Motor.setDuty((float)motorSpeeds[4]);
    BR_Motor.setDuty((float)motorSpeeds[5]);
}