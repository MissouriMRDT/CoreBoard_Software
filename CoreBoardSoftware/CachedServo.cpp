#include "CachedServo.h"

uint8_t CachedServo::attach(int pin, int minServo, int maxServo) { return m_servo.attach(pin, minServo, maxServo); }

void CachedServo::write() { 
    if(target != m_lastTarget) { // only send if changed
        // clamp target to range
        if(target < m_minAngle) 
            target = m_minAngle;
        if(target > m_maxAngle)
            target = m_maxAngle;
        m_servo.write(target); 
    }
    m_lastTarget = target;
}

void CachedServo::write(int16_t angle) {
    target = angle;
    write();
}
