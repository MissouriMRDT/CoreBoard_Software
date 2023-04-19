#include "CoreBoardSoftware.h"

void setup() {

    // Start RoveComm
    //TODO
    RoveComm.begin();
    delay(100);

    autonomy.begin();
    autonomy.setBrightness(BRIGHTNESS);
    
    // Initialize debug serial port
    Serial.begin(9600);
    Serial.println("Serial init");
    Serial.println("RoveComm Init");
    
    // attaches the servo array to the respective pins (1-8 servos are 0-7)
    servos[RD_SERVO].attach(RD_SERVO_PIN);
    servos[RMP_SERVO].attach(RMP_SERVO_PIN);
    servos[RMT_SERVO].attach(RMT_SERVO_PIN);
    servos[LD_SERVO].attach(LD_SERVO_PIN);
    servos[LMP_SERVO].attach(LMP_SERVO_PIN);
    servos[LMT_SERVO].attach(LMT_SERVO_PIN);
    servos[SPARE1_SERVO].attach(SPARE1_SERVO);

    // servo positions are set to starting positions
    servoPosition[RD_SERVO] = SERVO1_START;
    servoPosition[RMP_SERVO] = SERVO2_START;
    servoPosition[RMT_SERVO] = SERVO3_START;
    servoPosition[LD_SERVO] = SERVO4_START;
    servoPosition[LMP_SERVO] = SERVO5_START;
    servoPosition[LMT_SERVO] = SERVO6_START;
    servoPosition[SPARE1_SERVO] = SERVO7_START;

    // servo max values put in an array
    servoMax[RD_SERVO] = RD_SERVO_MAX;
    servoMax[RMP_SERVO] = RMP_SERVO_MAX;
    servoMax[RMT_SERVO] = RMT_SERVO_MAX;
    servoMax[LD_SERVO] = LD_SERVO_MAX;
    servoMax[LMP_SERVO] = LMP_SERVO_MAX;
    servoMax[LMT_SERVO] = LMT_SERVO_MAX;
    servoMax[SPARE1_SERVO] = SPARE1_MAX;

    // servo min values put in an array
    servoMin[RD_SERVO] = RD_SERVO_MIN;
    servoMin[RMP_SERVO] = RMP_SERVO_MIN;
    servoMin[RMT_SERVO] = RMT_SERVO_MIN;
    servoMin[LD_SERVO] = LD_SERVO_MIN;
    servoMin[LMP_SERVO] = LMP_SERVO_MIN;
    servoMin[LMT_SERVO] = LMT_SERVO_MIN;
    servoMin[SPARE1_SERVO] = SPARE1_MIN;

    pinMode(RD_BUTTON, INPUT);
    pinMode(RMP_BUTTON, INPUT);
    pinMode(RMT_BUTTON, INPUT);
    pinMode(LD_BUTTON, INPUT);
    pinMode(LMP_BUTTON, INPUT);
    pinMode(LMT_BUTTON, INPUT);
    pinMode(SPARE1_BUTTON, INPUT);

    delay(1000);
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        servos[i].write(servoMin[i]);
    }
    delay(1000);
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        servos[i].write(servoMax[i]);
    }
    delay(1000);
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        servos[i].write(servoPosition[i]);
    }
    }


    // Set override buttons to input
    for(int i = 0; i < 6; i++) {
        pinMode(motorButtons[i], INPUT);
    }

    lastRampTime = millis();

    watchdog.begin(EStop, WATCHDOG_TIME);
    

    // Initialize VESC serial ports
    FL_SERIAL.begin(115200);
    while(!(FL_SERIAL));
    
    ML_SERIAL.begin(115200);
    while(!(ML_SERIAL));
    
    BL_SERIAL.begin(115200);
    while(!(BL_SERIAL));
    
    FR_SERIAL.begin(115200);
    while(!(FR_SERIAL));
    
    MR_SERIAL.begin(115200);
    while(!(MR_SERIAL));
    
    BR_SERIAL.begin(115200);
    while(!(BR_SERIAL));

    FL_UART.setSerialPort(&FL_SERIAL);
    ML_UART.setSerialPort(&ML_SERIAL);
    BL_UART.setSerialPort(&BL_SERIAL);
    FR_UART.setSerialPort(&FR_SERIAL);
    MR_UART.setSerialPort(&MR_SERIAL);
    BR_UART.setSerialPort(&BR_SERIAL);
    
    telemetry.begin(Telemetry, TELEMETRY_UPDATE);

}

void loop() 
{

    data = (uint8_t*)packet.data;
    if(packet.data_id != 0)
    {
        switch(packet.data_id)
        {
            case RC_MULTIMEDIABOARD_LEDRGB_DATA_ID:
                autonomy.fill(autonomy.Color(data[0], data[1], data[2]));
                break;

            case RC_MULTIMEDIABOARD_STATEDISPLAY_DATA_ID:
                switch (data[0])
                {
                    case TELEOP:
                        autonomy.fill(autonomy.Color(0, 0, 255));
                        break;
                    
                    case AUTONOMY:
                        autonomy.fill(autonomy.Color(255, 0, 0));
                        break;

                    case REACHED_GOAL:
                        for(uint8_t i = 0; i < 5; i++)
                        {
                            autonomy.fill(autonomy.Color(0, 255, 0));
                            autonomy.show();
                            delay(500);
                            autonomy.clear();
                            autonomy.show();
                            delay(500);
                        }
                        break;
                }
                break;

            case RC_MULTIMEDIABOARD_BRIGHTNESS_DATA_ID:
                if(data[0] >= BRIGHTNESS)
                {
                    autonomy.setBrightness(BRIGHTNESS);
                }
                else
                {
                    autonomy.setBrightness(data[0]);
                }
                break;
        }
    }
    autonomy.show();

    // Read incoming packet
    packet = RoveComm.read();
    
    switch(packet.data_id) 
    {
        case RC_DRIVEBOARD_DRIVELEFTRIGHT_DATA_ID:
        {
            // Read packet and cast to correct type
            float* lrspeeds;
            lrspeeds = (float*)packet.data;

            // Map speed values from +-1000 to max and min rpm
            // TODO implement min drive rmp
            float leftSpeed = map(lrspeeds[0], -1000, 1000, -1, 1);
            float rightSpeed = map(lrspeeds[1], -1000, 1000, -1, 1);

            // Send RPM values to VESCs (First 3 are left, next 3 are right)
            for(int i = 0; i < 6; i++) 
                motorTargets[i] = (i < 3) ? leftSpeed : rightSpeed;

            watchdog.begin(EStop, WATCHDOG_TIME);

            FL_UART.setDuty((float)leftSpeed);
            ML_UART.setDuty((float)leftSpeed);
            BL_UART.setDuty((float)leftSpeed);
            FR_UART.setDuty((float)rightSpeed);
            MR_UART.setDuty((float)rightSpeed);
            BR_UART.setDuty((float)rightSpeed);
            break;
        }
        case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATA_ID:
        {
            // Read packet and cast to correct type
            float* speeds;
            speeds = (float*)packet.data;

            // Map speed values and send to VESCs
            for(int i = 0; i < 6; i++) 
                motorTargets[i] = map(speeds[i], -1000, 1000, -1, 1);

            watchdog.begin(EStop, WATCHDOG_TIME);
            break;
        }
    }
    

    // Override speeds if button is pressed
    maxRamp = (millis() - lastRampTime) * DRIVE_MAX_RAMP;
    for(int i = 0; i < 6; i++)
    {
        if(digitalRead(motorButtons[i]))
        {
          //Invert motor output if switch is toggled
          if(digitalRead(DIR_SWITCH))
          {
            motorTargets[i] = -BUTTON_OVERIDE_SPEED;
          } 
         else 
          {
            motorTargets[i] = BUTTON_OVERIDE_SPEED;
          }
        }

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


    if (digitalRead(RD_BUTTON)) {
        Serial.println("RD_BUTTON");
        incrementGimbal(RD_SERVO, BUTTON_INC_VAL);
    }
    
    if (digitalRead(RMP_BUTTON)) {
        Serial.println("RMP_BUTTON");
        incrementGimbal(RMP_SERVO, BUTTON_INC_VAL);
    }
    
    if (digitalRead(RMT_BUTTON)) {
        Serial.println("RMT_BUTTON");
        incrementGimbal(RMT_SERVO, BUTTON_INC_VAL);
    }
    
    if (digitalRead(LD_BUTTON)) {
        Serial.println("LD_BUTTON");
        incrementGimbal(LD_SERVO, BUTTON_INC_VAL);
    }
    
    if (digitalRead(LMP_BUTTON)) {
        Serial.println("LMP_BUTTON");
        incrementGimbal(LMP_SERVO, BUTTON_INC_VAL);
    }
    
    if (digitalRead(LMT_BUTTON)) {
        Serial.println("LMT_BUTTON");
        incrementGimbal(LMT_SERVO, BUTTON_INC_VAL);
    }
    
    if (digitalRead(SPARE1_BUTTON)) {
        Serial.println("SPARE1_BUTTON");
        incrementGimbal(SPARE1_SERVO, BUTTON_INC_VAL);
    }
    }

    switch (packet.data_id) {

        // Increment left drive gimbal by [-180, 180]
        case RC_GIMBALBOARD_LEFTDRIVEGIMBALINCREMENT_DATA_ID:
        {
            int16_t inc = ((int16_t*) packet.data)[0];
            incrementGimbal(LD_SERVO, inc);
            break;
        }

        // Increment right drive gimbal by [-180, 180]
        case RC_GIMBALBOARD_RIGHTDRIVEGIMBALINCREMENT_DATA_ID:
        {
            int16_t inc = ((int16_t*) packet.data)[0];
            incrementGimbal(RD_SERVO, inc);
            break;
        }

        // Increment left pan and tilt gimbals by [-180, 180]
        case RC_GIMBALBOARD_LEFTMAINGIMBALINCREMENT_DATA_ID:
        {
            int16_t* inc = (int16_t*) packet.data;
            incrementGimbal(LMP_SERVO, inc[0]);
            incrementGimbal(LMT_SERVO, inc[1]);
            break;
        }

        // Increment right pan and tilt gimbals by [-180, 180]
        case RC_GIMBALBOARD_RIGHTMAINGIMBALINCREMENT_DATA_ID:
        {
            int16_t* inc = (int16_t*) packet.data;
            incrementGimbal(RMP_SERVO, inc[0]);
            incrementGimbal(RMT_SERVO, inc[1]);
            break;
        }

        //
        default:
            break;
    }
    
    
    lastRampTime = millis();

    FL_UART.setDuty((float)motorSpeeds[0]);
    ML_UART.setDuty((float)motorSpeeds[1]);
    BL_UART.setDuty((float)motorSpeeds[2]);
    FR_UART.setDuty((float)motorSpeeds[3]);
    MR_UART.setDuty((float)motorSpeeds[4]);
    BR_UART.setDuty((float)motorSpeeds[5]);
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
    if(FL_UART.getVescValues()) {
        motorCurrent[0] = (float)map(FL_UART.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[0] = 0;
    }


    if(ML_UART.getVescValues()) {
        motorCurrent[1] = (float)map(ML_UART.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[1] = 0;
    }


    if(BL_UART.getVescValues()) {
        motorCurrent[2] = (float)map(BL_UART.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[2] = 0;
    }

    
    if(FR_UART.getVescValues()) {
        motorCurrent[3] = (float)map(FR_UART.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[3] = 0;
    }


    if(MR_UART.getVescValues()) {
        motorCurrent[4] = (float)map(MR_UART.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[4] = 0;
    }


    if(BR_UART.getVescValues()) {
        motorCurrent[5] = (float)map(BR_UART.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[5] = 0;
    }

    RoveComm.write(RC_DRIVEBOARD_DRIVESPEEDS_DATA_ID, RC_DRIVEBOARD_DRIVESPEEDS_DATA_COUNT, motorCurrent);
}

void incrementGimbal (const int &id, const int16_t &incrementVal)
{
    if (digitalRead(RIGHT_SWITCH)) {
        servoPosition[id] += incrementVal;
    }
    else if (digitalRead(LEFT_SWITCH)) {
        servoPosition[id] -= incrementVal;
    }
    Serial.print("ServoPosition[id]:");
    Serial.print(servoPosition[id]);
    Serial.print(" ");
    if (servoPosition[id] > servoMax[id]) {
        servoPosition[id] = servoMax[id];
    }
    else if (servoPosition[id] < servoMin[id]) {
        servoPosition[id] = servoMin[id];
    }

    Serial.print("new value:");
    Serial.println(servoPosition[id]);

    servos[id].write(servoPosition[id]);
}