#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Wire.h>

class Accelerometer {

public:
    Accelerometer(uint8_t sda, uint8_t scl) : m_sdaPin(sda), m_sclPin(scl) {}

    void begin();

    void read();

    float acceleration[3]; // m/s^2
    float temperature; // degrees C
    float gyro[3]; // deg/s

private:

    uint8_t m_sdaPin, m_sclPin;
    char m_buf[7];

};

#endif
