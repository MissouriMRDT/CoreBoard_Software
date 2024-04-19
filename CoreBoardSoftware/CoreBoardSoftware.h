#ifndef CoreBoardSoftware_h
#define CoreBoardSoftware_h

#include "PinAssignments.h"

#include <RoveComm.h>
#include <VescUart.h>

#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#define MAX_BRIGHTNESS          70
#define LED_COUNT               256

#define DRIVE_MIN_RPM           2000
#define DRIVE_MAX_RPM           30000
#define DRIVE_MAX_RAMP          0.005

#define LEFT_DRIVE_MIN          0
#define LEFT_PAN_MIN            0
#define LEFT_TILT_MIN           0
#define RIGHT_DRIVE_MIN         0
#define RIGHT_PAN_MIN           0
#define RIGHT_TILT_MIN          0
#define BACK_DRIVE_MIN          0

#define LEFT_DRIVE_MAX          180
#define LEFT_PAN_MAX            180
#define LEFT_TILT_MAX           180
#define RIGHT_DRIVE_MAX         180
#define RIGHT_PAN_MAX           180
#define RIGHT_TILT_MAX          180
#define BACK_DRIVE_MAX          180

#define TELEMETRY_UPDATE        150000
IntervalTimer telemetry;

#define WATCHDOG_TIMEOUT_TELEOP         300000
#define WATCHDOG_TIMEOUT_AUTONOMY       1500000
IntervalTimer watchdog;
bool watchdogOverride = false;
uint8_t watchdogMode = 0; // 0: Teleop, 1: Autonomy


//Rovecomm Declaration
EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);
RoveCommEthernet RoveComm;
rovecomm_packet packet;

uint32_t lastTimestamp;

Adafruit_NeoPixel neoPixel = Adafruit_NeoPixel(LED_COUNT, NEOPIXEL);

//Vesc Serial Declaration
VescUart FL_Motor;
VescUart FR_Motor;
VescUart ML_Motor;
VescUart MR_Motor;
VescUart BL_Motor;
VescUart BR_Motor;

//All wheels are in order of FL, ML, BL, FR, MR, BR
float motorTargets[6] = {0, 0, 0, 0, 0, 0};
float motorSpeeds[6] = {0, 0, 0, 0, 0, 0};
int16_t motorCurrent[6] = {0, 0, 0, 0, 0, 0};

// A thin abstraction around an arduino Servo.
// Set the public "target" field, then send it with write().
// You can also write(value) which sets target and then write()'s.
class CachedServo {
  public:
    CachedServo(int16_t target, int minTarget, int maxTarget): target(target), m_minTarget(minTarget), m_maxTarget(maxTarget) {}
    uint8_t attach(int pin, int minServo, int maxServo) { return m_servo.attach(pin, minServo, maxServo); }
    void write() { 
      if(target != m_lastTarget) { // only send if changed
        // clamp target to range
        if(target < m_minTarget) 
          target = m_minTarget;
        if(target > m_maxTarget)
          target = m_maxTarget;
        m_servo.write(target); 
      }
      m_lastTarget = target;
    }
    void write(int16_t value) {
      target = value;
      write();
    }
    int16_t target;
  private:
    Servo m_servo;
    int16_t m_lastTarget = 0;
    int16_t m_minTarget, m_maxTarget;
};

//Servo Declarations - Three 9-pin Connectors each with Three Servos
CachedServo leftDriveServo(90, LEFT_DRIVE_MIN, LEFT_DRIVE_MAX);
CachedServo leftPanServo(90, LEFT_PAN_MIN, LEFT_PAN_MAX);
CachedServo leftTiltServo(90, LEFT_TILT_MIN, LEFT_TILT_MAX);

CachedServo rightDriveServo(90, RIGHT_DRIVE_MIN, RIGHT_DRIVE_MAX);
CachedServo rightPanServo(90, RIGHT_PAN_MIN, RIGHT_PAN_MAX);
CachedServo rightTiltServo(90, RIGHT_TILT_MIN, RIGHT_TILT_MAX);

CachedServo backDriveServo(90, BACK_DRIVE_MIN, BACK_DRIVE_MAX);
CachedServo servo8(90, 10, 160), servo9(90, 10, 160);

//Buttons Declaration
uint8_t lastManualButtons = 0;

// Methods
void estop();
void servoStartups();
void manualButtons();

#endif