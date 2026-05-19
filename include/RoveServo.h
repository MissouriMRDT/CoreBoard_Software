#ifndef ROVE_SERVO_H
#define ROVE_SERVO_H

#include <inttypes.h>

#if !defined(TEENSYDUINO)
#error "Only works on teensy"
#endif

class RoveServo
{
  private:
    uint8_t pin;
    int16_t angle;       // in degrees
    int16_t minAngle;    // 0 unless otherwise specified
    int16_t maxAngle;    // 180 unless otherwise specified
    int16_t minSoftLimit;
    int16_t maxSoftLimit;
    uint8_t min16;       // minimum pulse, 16uS units  (default is 34)
    uint8_t max16;       // maximum pulse, 16uS units, 0-4ms range (default is 150)
    static uint32_t attachedpins[]; // 1 bit per digital pin

  public:
    RoveServo();
    // pulse length for 0 degrees in microseconds, 544uS default
    // pulse length for 180 degrees in microseconds, 2400uS default
    uint8_t attach(int pinArg) { return attach(pinArg, 544, 2400); }
    // attach to a pin, sets pinMode, returns 0 on failure, won't
    // position the servo until a subsequent write() happens
    // Only works for 9 and 10.
    uint8_t attach(int pinArg, int min, int max);
    
    void configAngleRange(int16_t min, int16_t max); // default 0 to 180
    void configSoftLimits(int16_t min, int16_t max); // default min angle to max angle
    void disableSoftLimits(); // same as configSoftLimits(0, 0);
    bool softLimitsEnabled() const; // check if soft limits are enabled
    // void detach();
    void write(int angleArg); // specify the angle in degrees, 0 to 180
    int16_t read() { return angle; }
    uint8_t attached();
};

#endif // ROVE_SERVO_H
