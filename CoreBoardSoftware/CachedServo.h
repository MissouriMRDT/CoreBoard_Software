#ifndef CACHED_SERVO_H
#define CACHED_SERVO_H

// https://github.com/PaulStoffregen/PWMServo.git
// This library is necessary because the Arduino <Servo.h> library does not actually use the PWM pin,
// so when Interrupts and Serial hold up the processor, the gimbals twitch. This library calls analogWrite()
// so the twitching doesn't happen.
#include <PWMServo.h>

// A thin abstraction around an arduino Servo.
// Set the public "target" field, then send it with write().
// You can also write(value) which sets target and then write()'s.
class CachedServo {

public:

    CachedServo(int16_t target, int minAngle = 0, int maxAngle = 180): target(target), m_minAngle(minAngle), m_maxAngle(maxAngle) {}
    
    uint8_t attach(int pin, int minServo = 500, int maxServo = 2500);
    
    void write();
    
    void write(int16_t angle);
    
    int16_t target;

private:
  
    PWMServo m_servo;
    int16_t m_lastTarget = 0;
    int16_t m_minAngle, m_maxAngle;

};

#endif
