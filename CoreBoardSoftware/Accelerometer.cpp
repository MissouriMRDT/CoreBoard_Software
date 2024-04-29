#include "Accelerometer.h"

// Register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
const uint8_t MPU_ADDR = 0b1101000; // I2C address of MPU-6050

void Accelerometer::begin() {
    Wire.setSDA(m_sdaPin);
    Wire.setSCL(m_sclPin);
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR); // I2C
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to 0 to wake up MPU
    Wire.endTransmission(true);
}

/***********************************************************************************************************************************
 * Data format: is 3*int16_t + 1*int16_t + 3*int16_t = 14 bytes
 * Master | START | ADDR (0x3B) | WR (0) |     | START | ADDR | RD (1) |     |                | ACK |                | NACK | STOP |
 * Slave  |       |             |        | ACK |       |      |        | ACK | DATA (7 bytes) |     | DATA (7 bytes) |      |      |
 ***********************************************************************************************************************************/

void Accelerometer::read() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // start with register ACCEL_XOUT_H
    Wire.endTransmission(false); // keep connection alive
    Wire.requestFrom(MPU_ADDR, 7*2, true); // request next 14 registers

    for (int i = 0; i < 3; i++) { // acceleration: refer to section 4.17
        int16_t inVal = Wire.read() << 8 | Wire.read();
        acceleration[i] = (float)inVal / 16384 / 9.81; // convert to g, then to m/s^2
    }
    { // temperature: refer to section 4.18
        int16_t inVal = Wire.read() << 8 | Wire.read();
        temperature = (float)inVal / 340 + 36.53; // convert to C
    }
    for (int i = 0; i < 3; i++) { // rotational velocity: refer to section 4.19
        int16_t inVal = Wire.read() << 8 | Wire.read();
        gyro[i] = (float)inVal / 131; // convert to deg/s
    }
}