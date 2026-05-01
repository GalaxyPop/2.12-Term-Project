#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "imu.h"
#include "robot_pinout.h"
#include "robot_motion_control.h"


// USE ROLL FOR 'alpha' (tilt on the ramp -- neg sign makes pos alpha on way up, neg on way down)
// USE YAW FOR turn updating (increase to left, decrease to right);
// since it's relative, maybe reset to zero frequently or before turning (or just use differences -- uglier though)
// No need for pitch, but needs +4 offset

euler_t ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;


void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setupIMU() {

  bno08x.begin_SPI(BNO08X_CS, BNO08X_INT);

  setReports(reportType, reportIntervalUs);

  Serial.println("IMU Setup!");
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= -RAD_TO_DEG; //neg sign makes pos alpha on way up, neg on way down
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void scanIMU() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
    // bno08x.enableReport(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long last = 0;
    // long now = micros();
    // Serial.print(now - last);             Serial.print("\t");
    // last = now;
    // Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    // Serial.println("yaw: \t\t roll: \n\r");
    Serial.println("turn-angle: \t tilt-angle: \n\r");
    Serial.print(ypr.yaw);                Serial.print("\t\t");
    // Serial.print(ypr.pitch+4);    Serial.print("\t");
    // Serial.println("roll:");
    Serial.println(ypr.roll);
  }

}
