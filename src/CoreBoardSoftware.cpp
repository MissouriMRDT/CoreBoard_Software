#include <Arduino.h>
#include "CoreBoardSoftware.h"



void handleAxis(PWMServo& pan, PWMServo& tilt){
    if (!digitalRead(DIR_FORWARD)){
        tilt.write(tilt.read() + 1);
    }
    if (!digitalRead(DIR_BACK)){
        tilt.write(tilt.read() - 1);
    }
    if (!digitalRead(DIR_RIGHT)){
        pan.write(pan.read() + 1);
    }
    if (!digitalRead(DIR_LEFT)){
        pan.write(pan.read() - 1);
    }
}

    
void setup(){

    Spare1.attach(SPARE_1_SERVO);
    Spare2.attach(SPARE_2_SERVO);
    LeftPan.attach(LEFT_PAN_SERVO);
    LeftTilt.attach(LEFT_TILT_SERVO);
    BackPan.attach(BACK_PAN_SERVO);
    BackTilt.attach(BACK_TILT_SERVO);
    RightPan.attach(RIGHT_PAN_SERVO);
    RightTilt.attach(RIGHT_TILT_SERVO);

    pinMode(RTRY_1, INPUT_PULLDOWN);
    pinMode(RTRY_2, INPUT_PULLDOWN);
    pinMode(RTRY_4, INPUT_PULLDOWN);

    pinMode(DIR_FORWARD, INPUT_PULLUP);
    pinMode(DIR_BACK, INPUT_PULLUP);
    pinMode(DIR_RIGHT, INPUT_PULLUP);
    pinMode(DIR_LEFT, INPUT_PULLUP);

    RoveComm.begin(RC_COREBOARD_IPADDRESS);
}

void loop(){
    int mode = digitalRead(RTRY_1) | (digitalRead(RTRY_2) << 1) | (digitalRead(RTRY_4) << 2);
    switch (mode){
        case 0:
            break;
        case 1:
            handleAxis(LeftPan, LeftTilt);
            break;
        case 2:
            handleAxis(Spare1, Spare1);
            break;
        case 3:
            handleAxis(BackPan, BackTilt);
            break;
        case 4:
            handleAxis(Spare2, Spare2);
            break;
        case 5:
            handleAxis(RightPan, RightTilt);
            break;
        case 6:
            //wheels
            break;
        case 7:
            break;
    }


    //responding to commands
    RoveCommPacket packet;
    RoveComm.read(packet);
    switch (packet.dataId) {
        case RC_COREBOARD_DRIVELEFTRIGHT_DATA_ID:
            for (int i = 0; i < 3; i++) {
                wheelSpeeds[i] = packet.fdata[0];
                wheelSpeeds[i + 3] = packet.fdata[1];
            }
            break;
        case RC_COREBOARD_DRIVEINDIVIDUAL_DATA_ID:
            for (int i = 0; i < 6; i++) {
                wheelSpeeds[i] = packet.fdata[i];
            }
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
    }

    //telemetr

    FL_Motor.drive(1000 * wheelSpeeds[0]);
    ML_Motor.drive(1000 * wheelSpeeds[1]);
    BL_Motor.drive(1000 * wheelSpeeds[2]);
    FR_Motor.drive(1000 * wheelSpeeds[3]);
    MR_Motor.drive(1000 * wheelSpeeds[4]);
    BR_Motor.drive(1000 * wheelSpeeds[5]);

}
