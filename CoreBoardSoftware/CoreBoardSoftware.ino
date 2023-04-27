#include "CoreBoardSoftware.h"

void setup() {
    // Initialize debug serial port
    Serial.begin(9600);
    Serial.println("Serial init");

    //Attach Servos to Pins
    leftDriveServo.attach(SERVO_1);
    leftPanServo.attach(SERVO_2);
    leftTiltServo.attach(SERVO_3);
    rightDriveServo.attach(SERVO_4);
    rightPanServo.attach(SERVO_5);
    rightTiltServo.attach(SERVO_6);
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
    
    //Start RoveComm
    RoveComm.begin(RC_COREBOARD_FIRSTOCTET, RC_COREBOARD_SECONDOCTET, RC_COREBOARD_THIRDOCTET, RC_COREBOARD_FOURTHOCTET, &TCPServer);
 
    telemetry.begin(Telemetry, TELEMETRY_UPDATE);

    servoStartups();
    
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
            int16_t data = ((int16_t) packet.data[0]);
            servoTargets[0] += data;
            break;
        }

        // Increment right drive gimbal by [-180, 180]
        case RC_GIMBALBOARD_RIGHTDRIVEGIMBALINCREMENT_DATA_ID:
        {
            int16_t data = ((int16_t) packet.data[0]);
            servoTargets[3] += data;
            break;
        }

        // Increment left pan and tilt gimbals by [-180, 180]
        case RC_GIMBALBOARD_LEFTMAINGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            servoTargets[1] += data[0];
            servoTargets[2] += data[1];
            break;
        }

        // Increment right pan and tilt gimbals by [-180, 180]
        case RC_GIMBALBOARD_RIGHTMAINGIMBALINCREMENT_DATA_ID:
        {
            int16_t* data = (int16_t*) packet.data;
            servoTargets[4] += data[0];
            servoTargets[5] += data[1];
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
            float* data;
            data = (float*)packet.data;

            float leftSpeed = data[0];
            float rightSpeed = data[1];

            for(int i = 0; i < 6; i++) 
                motorTargets[i] = (i < 3) ? leftSpeed : rightSpeed;

            watchdog.begin(EStop, WATCHDOG_TIME);
            break;
        }

        //Set All individual Motors to a DutyCycle [-1, 1]
        case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATA_ID:
        {
            float* data;
            data = (float*)packet.data;

            for(int i = 0; i < 6; i++) 
                motorTargets[i] = data[i];

            watchdog.begin(EStop, WATCHDOG_TIME);
            break;
        }
    }

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

    manualButtons();

    FL_Motor.setDuty(motorSpeeds[0]);
    ML_Motor.setDuty(motorSpeeds[1]);
    BL_Motor.setDuty(motorSpeeds[2]);
    FR_Motor.setDuty(motorSpeeds[3]);
    MR_Motor.setDuty(motorSpeeds[4]);
    BR_Motor.setDuty(motorSpeeds[5]);

    leftDriveServo.write(servoTargets[0]);
    leftPanServo.write(servoTargets[1]);
    leftTiltServo.write(servoTargets[2]);
    rightDriveServo.write(servoTargets[3]);
    rightPanServo.write(servoTargets[4]);
    rightTiltServo.write(servoTargets[5]);
    servo7.write(servoTargets[6]);
    servo8.write(servoTargets[7]);
    servo9.write(servoTargets[8]);

    neoPixel.show();

    lastRampTime = millis();
}



void Telemetry()
{
    int16_t motorCurrent[6];

    //Converts Vesc RPM values to a value from [-DRIVE_MAX_RPM, DRIVE_MAX_RPM] -> [-1000, 1000]
    if(FL_Motor.getVescValues()) {
        motorCurrent[0] = map(FL_Motor.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[0] = 0;
    }


    if(ML_Motor.getVescValues()) {
        motorCurrent[1] = map(ML_Motor.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[1] = 0;
    }


    if(BL_Motor.getVescValues()) {
        motorCurrent[2] = map(BL_Motor.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[2] = 0;
    }

    
    if(FR_Motor.getVescValues()) {
        motorCurrent[3] = map(FR_Motor.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[3] = 0;
    }


    if(MR_Motor.getVescValues()) {
        motorCurrent[4] = map(MR_Motor.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[4] = 0;
    }


    if(BR_Motor.getVescValues()) {
        motorCurrent[5] = map(BR_Motor.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[5] = 0;
    }

    RoveComm.write(RC_DRIVEBOARD_DRIVESPEEDS_DATA_ID, RC_DRIVEBOARD_DRIVESPEEDS_DATA_COUNT, motorCurrent);
}



void manualButtons()
{
    bool reverse = digitalRead(REVERSE);
    uint8_t manualButtons = (digitalRead(B_ENC_3)<<3) | (digitalRead(B_ENC_2)<<2) | (digitalRead(B_ENC_1)<<1) | (digitalRead(B_ENC_0)<<0);

    if(manualButtons == lastManualButtons){
        switch(manualButtons){
            case 1: //FR
                motorSpeeds[3] = (reverse? -0.5 : 0.5);
                break;
            
            case 2: //FM (MR)
                motorSpeeds[4] = (reverse? -0.5 : 0.5);
                break;
            
            case 3: //FL
                motorSpeeds[0] = (reverse? -0.5 : 0.5);
                break;
            
            case 4: //BR
                motorSpeeds[5] = (reverse? -0.5 : 0.5);
                break;
            
            case 5: //BM (ML)
                motorSpeeds[1] = (reverse? -0.5 : 0.5);
                break;
            
            case 6: //BL
                motorSpeeds[2] = (reverse? -0.5 : 0.5);
                break;

            case 7: //S1
                servoTargets[0] += (reverse? -2 : 2);
                if(servoTargets[0] > 180) servoTargets[0] = 180;
                if(servoTargets[0] < 0) servoTargets[0] = 0;
                delay(15);
                break;
            
            case 8: //S2
                servoTargets[1] += (reverse? -2 : 2);
                if(servoTargets[1] > 180) servoTargets[1] = 180;
                if(servoTargets[1] < 0) servoTargets[1] = 0;
                delay(15);
                break;
            
            case 9: //S3
                servoTargets[2] += (reverse? -2 : 2);
                if(servoTargets[2] > 180) servoTargets[2] = 180;
                if(servoTargets[2] < 0) servoTargets[2] = 0;
                delay(15);
                break;
            
            case 10: //S4
                servoTargets[3] += (reverse? -2 : 2);
                if(servoTargets[3] > 180) servoTargets[3] = 180;
                if(servoTargets[3] < 0) servoTargets[3] = 0;
                delay(15);
                break;
            
            case 11: //S5
                servoTargets[4] += (reverse? -2 : 2);
                if(servoTargets[4] > 180) servoTargets[4] = 180;
                if(servoTargets[4] < 0) servoTargets[4] = 0;
                delay(15);
                break;
            
            case 12: //S6
                servoTargets[5] += (reverse? -2 : 2);
                if(servoTargets[5] > 180) servoTargets[5] = 180;
                if(servoTargets[5] < 0) servoTargets[5] = 0;
                delay(15);
                break;
            
            case 13: //S7
                servoTargets[6] += (reverse? -2 : 2);
                if(servoTargets[6] > 180) servoTargets[6] = 180;
                if(servoTargets[6] < 0) servoTargets[6] = 0;
                delay(15);
                break;
            
            case 14: //S8
                servoTargets[7] += (reverse? -2 : 2);
                if(servoTargets[7] > 180) servoTargets[7] = 180;
                if(servoTargets[7] < 0) servoTargets[7] = 0;
                delay(15);
                break;
            
            case 15: //S9
                servoTargets[8] += (reverse? -2 : 2);
                if(servoTargets[8] > 180) servoTargets[8] = 180;
                if(servoTargets[8] < 0) servoTargets[8] = 0;
                delay(15);
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
                break;

            case 8: //S2
                break;
            
            case 9: //S3
                break;
            
            case 10: //S4
                break;
            
            case 11: //S5
                break;
            
            case 12: //S6
                break;
            
            case 13: //S7
                break;
            
            case 14: //S8
                break;
            
            case 15: //S9
                break;

            default:
                break;
        }
    }

    lastManualButtons = manualButtons;
}

void servoStartups()
{
    delay(1000);
    leftDriveServo.write(SERVO_1_MIN);
    leftPanServo.write(SERVO_2_MIN);
    leftTiltServo.write(SERVO_3_MIN);
    rightDriveServo.write(SERVO_4_MIN);
    rightPanServo.write(SERVO_5_MIN);
    rightTiltServo.write(SERVO_6_MIN);
    servo7.write(SERVO_7_MIN);
    servo8.write(SERVO_8_MIN);
    servo9.write(SERVO_9_MIN);

    delay(1000);
    leftDriveServo.write(SERVO_1_MAX);
    leftPanServo.write(SERVO_2_MAX);
    leftTiltServo.write(SERVO_3_MAX);
    rightDriveServo.write(SERVO_4_MAX);
    rightPanServo.write(SERVO_5_MAX);
    rightTiltServo.write(SERVO_6_MAX);
    servo7.write(SERVO_7_MAX);
    servo8.write(SERVO_8_MAX);
    servo9.write(SERVO_9_MAX);

    delay(1000);
    leftDriveServo.write(servoTargets[0]);
    leftPanServo.write(servoTargets[1]);
    leftTiltServo.write(servoTargets[2]);
    rightDriveServo.write(servoTargets[3]);
    rightPanServo.write(servoTargets[4]);
    rightTiltServo.write(servoTargets[5]);
    servo7.write(servoTargets[6]);
    servo8.write(servoTargets[7]);
    servo9.write(servoTargets[8]);
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
