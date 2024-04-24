#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Wire.h>

struct Vector3 {
    int16_t x, y, z;
};

class Accelerometer {

public:
    Accelerometer(uint8_t sda, uint8_t scl) : m_sdaPin(sda), m_sclPin(scl) {}

    void begin();

    void read();

    Vector3 acceleration;
    int16_t temperature;
    Vector3 gyro;
    //Vector3 magnetometer:

private:

    uint8_t m_sdaPin, m_sclPin;
    char m_buf[7];

};

#endif