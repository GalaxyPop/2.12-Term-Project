#include <Arduino.h>
#include "robot_drive.h"
#include "wireless.h"
#include "util.h"
#include "robot_motion_control.h"
#include "kinematics.h"
#include "robot_autonomous.h"

extern TaskSpace targetXY;
extern bool armPositionControl; // if true, arm's position will be controlled by joystick inputs

void setup() {
    Serial.begin();
    setupDrive();
    setupWireless();
    setupAutonomous();
}

void loop() {
    // Update velocity setpoints based on trajectory at 50Hz
    EVERY_N_MILLIS(20) {
        followTrajectory();
    }

    // Update PID at 200Hz
    EVERY_N_MILLIS(5) {
        updatePIDs(armPositionControl);
    }

    // Send and print robot values at 20Hz
    EVERY_N_MILLIS(50) {
        updateOdometry();
        sendRobotData();
    }

}
