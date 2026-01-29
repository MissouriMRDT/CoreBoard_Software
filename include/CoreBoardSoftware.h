#ifndef COREBOARD_SOFTWARE_H
#define COREBOARD_SOFTWARE_H

#include "PinAssignments.h"
#include "Accelerometer.h"

#include <RoveComm.h>
#include <RoveVESC.h>

#include <Adafruit_NeoPixel.h>
#include <PWMServo.h>

//[FL, ML, BL, FR, MR, BR]
float wheelSpeeds[6] = {0};

RoveVESC FL_Motor(&FL_SERIAL);
RoveVESC ML_Motor(&ML_SERIAL);
RoveVESC BL_Motor(&BL_SERIAL);
RoveVESC FR_Motor(&FR_SERIAL);
RoveVESC MR_Motor(&MR_SERIAL);
RoveVESC BR_Motor(&BR_SERIAL);

PWMServo Spare1;
PWMServo Spare2;
PWMServo LeftPan;
PWMServo LeftTilt;
PWMServo BackPan;
PWMServo BackTilt;
PWMServo RightPan;
PWMServo RightTilt;

RoveCommEthernet RoveComm;

#endif