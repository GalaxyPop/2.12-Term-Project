#include <Arduino.h>
#include "robot_drive.h"
#include "wireless.h"
#include "util.h"
#include "robot_motion_control.h"
#include "kinematics.h"
#include "robot_autonomous.h"
#include "jetson_link.h"
#include "robot_pinout.h"
#include "mcpwm.h"
#include "servo_control.h"

extern TaskSpace targetXY;

void setup() {
    // Serial.begin(921600);
    // USB CDC on ESP32-S3: wait for host or time out after 3 s.
    unsigned long t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) {
        delay(10);
    }
    setupDrive();
    setupWireless();
    setupDrive();
    setupServo();
    delay(1000);
    gripClose();
    setupAutonomous();
    setupJetsonLink();
}

void loop() {
    // Must run every iteration — at 921600 the USB-CDC ring fills fast,
    // and gating the reader behind EVERY_N_MILLIS drops bytes mid-packet.
    handleJetsonSerial();

    EVERY_N_MILLIS(20) {
        scanIMU();
        followTrajectory();
    }

    EVERY_N_MILLIS(5) {
        updatePIDs();
    }

    EVERY_N_MILLIS(50) {
        updateOdometry();
        sendRobotData();
    }

    EVERY_N_MILLIS(20) {
        sendJetsonTelemetry();
    }
}
