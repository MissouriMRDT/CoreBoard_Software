#include "Arduino.h"
#include "RoveServo.h"

/*
  RoveServo.cpp - Hardware Servo Timer Library
  http://arduiniana.org/libraries/RoveServo/
  Author: Jim Studt, jim@federated.com
  Copyright (c) 2007 David A. Mellis.  All right reserved.
  renamed to RoveServo by Mikal Hart
  ported to other chips by Paul Stoffregen

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define NO_ANGLE (0xff)

uint32_t RoveServo::attachedpins[(NUM_DIGITAL_PINS+31)/32]; // 1 bit per digital pin

RoveServo::RoveServo() : pin(255), angle(NO_ANGLE) {
    configAngleRange(0, 180);
    disableSoftLimits();
}

uint8_t RoveServo::attach(int pinArg, int min, int max) {
	//Serial.printf("attach, pin=%d, min=%d, max=%d\n", pinArg, min, max);
	if (pinArg < 0 || pinArg >= NUM_DIGITAL_PINS) return 0;
	if (!digitalPinHasPWM(pinArg)) return 0;
	pin = pinArg;
	analogWriteFrequency(pin, 50);
	min16 = min >> 4;
	max16 = max >> 4;
	angle = NO_ANGLE;
	digitalWrite(pin, LOW);
	pinMode(pin, OUTPUT);
	attachedpins[pin >> 5] |= (1 << (pin & 31));
	return 1;
}

void RoveServo::configAngleRange(int16_t min, int16_t max) {
    minAngle = min;
    maxAngle = max;
    if (softLimitsEnabled()) {
        minSoftLimit = constrain(minSoftLimit, minAngle, maxAngle);
        maxSoftLimit = constrain(maxSoftLimit, minSoftLimit, maxAngle);
    }
}

void RoveServo::configSoftLimits(int16_t min, int16_t max) {
    minSoftLimit = constrain(min, minAngle, maxAngle);
    maxSoftLimit = constrain(max, minSoftLimit, maxAngle);
}

void RoveServo::disableSoftLimits() {
    configSoftLimits(0, 0);
}

bool RoveServo::softLimitsEnabled() const {
    return !(minSoftLimit == 0 && maxSoftLimit == 0);
}


void RoveServo::write(int angleArg) {
	//Serial.printf("write, pin=%d, angle=%d\n", pin, angleArg);
	if (pin >= NUM_DIGITAL_PINS) return;
    if (softLimitsEnabled()) {
        angleArg = constrain(angleArg, minSoftLimit, maxSoftLimit);
    } else {
        angleArg = constrain(angleArg, minAngle, maxAngle);
    }

	angle = angleArg;
	// uint32_t us = (((max16 - min16) * 46603 * angle) >> 11) + (min16 << 12); // us*256
	// uint32_t duty = (us * 3355) >> 22;
	float usec = (float)((max16 - min16)<<4) * ((float)(angle - minAngle) / (float)(maxAngle - minAngle)) + (float)(min16<<4);
	uint32_t duty = (int)(usec / 20000.0f * 4096.0f);
	//Serial.printf("angle=%d, usec=%.2f, us=%.2f, duty=%d, min=%d, max=%d\n",
    //angle, usec, (float)us / 256.0f, duty, min16<<4, max16<<4);
#if TEENSYDUINO >= 137
	noInterrupts();
	uint32_t oldres = analogWriteResolution(12);
	analogWrite(pin, duty);
	analogWriteResolution(oldres);
	interrupts();
#else
	analogWriteResolution(12);
	analogWrite(pin, duty);
#endif
}

uint8_t RoveServo::attached() {
	if (pin >= NUM_DIGITAL_PINS) return 0;
	return (attachedpins[pin >> 5] & (1 << (pin & 31))) ? 1 : 0;
}
