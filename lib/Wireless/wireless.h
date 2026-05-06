#ifndef WIRELESS_H
#define WIRELESS_H

#include <esp_now.h>
#include "joystick.h"
#include "dpad.h"
#include "display.h"


// EC:DA:3B:41:A3:C0 is the old address of the old controller
// EC:DA:3B:5C:85:1C is the new address
const uint8_t controllerAddr[] = {0xEC, 0xDA, 0x3B, 0x5C, 0x85, 0x1C};

// EC:DA:3B:5C:89:E4
const uint8_t robotAddr[] = {0xEC, 0xDA, 0x3B, 0x5C, 0x89, 0xE4};

struct ControllerMessage { //This struct is defined for a complex controller, but in lab 7 we only use a single joystick, so values are only written to joystick1.
    unsigned long millis;
    JoystickReading joystick1;
    JoystickReading joystick2;
    DPadReading dPad;
    bool buttonL;
    bool buttonR;
    TouchReading touchPoint;
    bool dpadUp;
    bool dpadDown;
    bool dpadLeft;
    bool dpadRight;
    bool dpadSelect;
    int32_t encoderPosition;

    void print();
    bool operator==(const ControllerMessage& other);
} ;

struct RobotMessage {
    unsigned long millis;
    float x;
    float y;
    float theta;

    float a; // x position of arm end effector in robot frame
    float b; // y position of arm end effector in robot frame

    void print();
    bool operator==(const RobotMessage& other);
} ;

void onSendData(const uint8_t * mac, esp_now_send_status_t status);
void onRecvData(const uint8_t * mac, const uint8_t *data, int len);
void setupWireless();
bool sendControllerData();
bool sendRobotData();

extern const uint8_t * peerAddr;
extern esp_now_peer_info_t peerInfo;

extern bool freshWirelessData;
extern ControllerMessage controllerMessage;
extern RobotMessage robotMessage;

#endif // WIRELESS_H
