// Minimal sketch: print the IMU roll angle the way robot_imu.cpp exposes it.
// Use this to verify that climbing the ramp produces a +ve reading and that
// the magnitude matches the chassis tilt — same sign convention the arm
// tray-leveling code in robot_motion_control.cpp consumes via ypr.roll.

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "robot_pinout.h"

static Adafruit_BNO08x bno08x(BNO08X_RESET);
static sh2_SensorValue_t sensorValue;

static const sh2_SensorId_t REPORT_TYPE = SH2_ARVR_STABILIZED_RV;
static const long REPORT_INTERVAL_US = 5000;

static float roll_deg = 0.0f;

static void setReports() {
    if (!bno08x.enableReport(REPORT_TYPE, REPORT_INTERVAL_US)) {
        Serial.println("Could not enable stabilized RV report");
    }
}

// Same math as robot_imu.cpp::quaternionToEuler, returning roll in degrees
// with the sign flip so +ve = climbing.
static float rollFromQuat(float qr, float qi, float qj, float qk) {
    float sqr = sq(qr), sqi = sq(qi), sqj = sq(qj), sqk = sq(qk);
    float roll_rad = atan2(2.0f * (qj * qk + qi * qr),
                           (-sqi - sqj + sqk + sqr));
    return roll_rad * -RAD_TO_DEG;
}

void setup() {
    Serial.begin(921600);
    unsigned long t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) delay(10);

    Serial.println("Roll-angle test starting");

    if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
        Serial.println("Failed to find BNO08x chip");
        while (1) delay(10);
    }
    Serial.println("BNO08x found");
    setReports();
}

void loop() {
    if (bno08x.wasReset()) {
        Serial.println("sensor was reset");
        setReports();
    }

    if (bno08x.getSensorEvent(&sensorValue) &&
        sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
        const auto &q = sensorValue.un.arvrStabilizedRV;
        roll_deg = rollFromQuat(q.real, q.i, q.j, q.k);
    }

    static unsigned long last_print = 0;
    if (millis() - last_print >= 50) {  // 20 Hz
        last_print = millis();
        Serial.printf("roll: %7.2f deg\n", roll_deg);
    }
}
