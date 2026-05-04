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

void writeServoUS(int pulse_us) {
    pulse_us = constrain(pulse_us, 1000, 2000);

    mcpwm_set_duty_in_us(
        MCPWM_UNIT_0,
        MCPWM_TIMER_0,
        MCPWM_OPR_A,
        pulse_us
    );
}

void setupServo() {
    // Route MCPWM0A to GPIO 43
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PIN);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;              // 50 Hz servo PWM
    pwm_config.cmpr_a = 0;                  // duty A starts at 0
    pwm_config.cmpr_b = 0;                  // unused
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void gripClose() {
    writeServoUS(GRIP_POS_US);
}

void gripOpen() {
    writeServoUS(OPEN_POS_US);
}

void setup() {
    Serial.begin(921600);
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

    setupServo();
    gripClose();
}

void loop() {
    
    // Update velocity setpoints based on trajectory at 50Hz
    EVERY_N_MILLIS(20) {
        scanIMU();
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

        Serial.printf("x: %.2f, y: %.2f, theta: %.2f\n",
                    robotMessage.x, robotMessage.y, robotMessage.theta,
                    robotMessage.a, robotMessage.b);
        Serial.printf("x: %.2f, y: %.2f\n",
                    targetXY.x, targetXY.y);
    }
