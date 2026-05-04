#include <Arduino.h>
#include "robot_drive.h"
#include "wireless.h"
#include "util.h"
#include "robot_motion_control.h"
#include "kinematics.h"
#include "robot_autonomous.h"
#include "jetson_link.h"

extern TaskSpace targetXY;

void setup() {
    Serial.begin(921600);
    // USB CDC on ESP32-S3: wait for host or time out after 3 s.
    unsigned long t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) {
        delay(10);
    }
    setupDrive();
    setupWireless();
    setupAutonomous();
    setupJetsonLink();
}

void loop() {
    // Must run every iteration — at 921600 the USB-CDC ring fills fast,
    // and gating the reader behind EVERY_N_MILLIS drops bytes mid-packet.
    handleJetsonSerial();

    // Update velocity setpoints based on trajectory at 50Hz
    EVERY_N_MILLIS(20) {
        followTrajectory();
    }

    // Update PID at 200Hz
    EVERY_N_MILLIS(5) {
        updatePIDs();
    }

    // Send and print robot values at 20Hz
    EVERY_N_MILLIS(50) {
        updateOdometry();
        sendRobotData();
    }

    // Telemetry to Jetson at 50Hz.
    EVERY_N_MILLIS(20) {
        sendJetsonTelemetry();
    }
}
