#include "servo_control.h"
#include <Arduino.h>
#include <ESP32Servo.h>

Servo trayServo;

const int SERVO_PIN    = 43;
const int GRIP_POS_US  = 2000;  // <<< TUNE: closed/grip position
const int OPEN_POS_US  = 1000;  // <<< TUNE: open position
const int NEUTRAL_US   = 1500;


void gripClose() {
    trayServo.writeMicroseconds(GRIP_POS_US);
}

void gripOpen() {
    trayServo.writeMicroseconds(OPEN_POS_US);
}

void setupServo() {
    ESP32PWM::allocateTimer(3);
    trayServo.setPeriodHertz(50);
    trayServo.attach(SERVO_PIN, 1000, 2000);
}
