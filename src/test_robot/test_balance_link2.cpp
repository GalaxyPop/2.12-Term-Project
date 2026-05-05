// Manual-tilt test for ramp tray-leveling.
//
// Holds link 1 vertical in the chassis frame and drives link 2 to stay
// horizontal in the world frame as the chassis pitches. Wheels are left at 0,
// so it's safe to lift the front of the robot by hand and watch link 2 rotate.
//
// Convention matches robot_motion_control.cpp tray-leveling:
//   theta1_chassis = THETA1_OFFSET (= pi/2)
//   theta2_chassis = -alpha   (alpha = chassis pitch, +ve climbing)
// so theta2_world = theta2_chassis + alpha = 0 (horizontal).
//
// Roll-angle nuance: ypr.roll is noisy at ~1 deg amplitude (e.g. reads 0.0
// level, then 0.9 after a lift/setdown cycle). A single-pole IIR low-pass
// filter is applied to alpha before it feeds the setpoint and the gravity
// feed-forward, so small sensor jitter doesn't drive the motors.
//
// Gravity comp is tilt-aware here: torque on each link depends on the link's
// orientation in the *world* frame, not the chassis frame. With chassis pitch
// alpha, theta_world = theta_chassis + alpha for each link, so cos() of the
// world-frame angle is the correct horizontal moment arm.

#include <Arduino.h>
#include <math.h>
#include <Adafruit_BNO08x.h>

#include "robot_pinout.h"
#include "robot_drive.h"      // Kp_arm, Ki_arm, Kd_arm, pidTau
#include "kinematics.h"       // L*, *_MASS_KG, GRAVITY, THETA*_OFFSET
#include "MotorDriver.h"
#include "PID.h"
#include "EncoderVelocity.h"

// ---------- IMU ----------
static Adafruit_BNO08x bno08x(BNO08X_RESET);
static sh2_SensorValue_t sensorValue;
static const sh2_SensorId_t REPORT_TYPE = SH2_ARVR_STABILIZED_RV;
static const long REPORT_INTERVAL_US = 5000;

static float roll_deg = 0.0f;        // last raw reading (sign-flipped: +ve = climbing)
static float alpha_filt_rad = 0.0f;  // low-passed chassis pitch in radians

// Single-pole IIR coefficient. With ~200 Hz IMU updates this gives ~50 ms tau.
static const float ROLL_LPF_BETA = 0.1f;

static void setReports() {
    if (!bno08x.enableReport(REPORT_TYPE, REPORT_INTERVAL_US)) {
        Serial.println("Could not enable stabilized RV report");
    }
}

// Same math as robot_imu.cpp::quaternionToEuler; sign flip so +ve = climbing
// keeps this in lockstep with the production tray-leveling convention.
static float rollFromQuat(float qr, float qi, float qj, float qk) {
    float sqr = sq(qr), sqi = sq(qi), sqj = sq(qj), sqk = sq(qk);
    float roll_rad = atan2(2.0f * (qj * qk + qi * qr),
                           (-sqi - sqj + sqk + sqr));
    return roll_rad * -RAD_TO_DEG;
}

// ---------- Arm hardware ----------
// Mirrors src/robot/robot_drive.cpp pinning, but only motor 0 (link 1) and
// motor 3 (link 2). Wheels are intentionally not initialized.
static MotorDriver motorL1{A_DIR1, A_PWM1, 4};
static MotorDriver motorL2{B_DIR2, B_PWM2, 7};

static EncoderVelocity encL1{ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_60_RPM, 0.2};
static EncoderVelocity encL2{ENCODER4_A_PIN, ENCODER4_B_PIN, CPR_60_RPM, 0.2};

static PID pidL1{Kp_arm_1, Ki_arm_1, Kd_arm_1, 0, pidTau_arm, false};
static PID pidL2{Kp_arm_2, Ki_arm_2, Kd_arm_2, 0, pidTau_arm, false};

static const double encoder_sign_L1 = -1.0;
static const double encoder_sign_L2 =  1.0;
static const double motor_sign_L1   = -1.0;
static const double motor_sign_L2   = -1.0;

static const double torqueToDuty1 = 0.05;
static const double torqueToDuty2 = 0.10;

static const double initial_pos_L1 = THETA1_OFFSET;
static const double initial_pos_L2 = THETA2_OFFSET;

// ---------- Tilt-aware gravity feed-forward ----------
// theta1, theta2 are absolute link angles in the *chassis* frame (what the
// encoders report). alpha is chassis pitch in radians. Adding alpha lifts the
// angles into the world frame so cos() picks up the true horizontal moment arm
// of gravity. When alpha = 0 this reduces to the existing computeGravity().
static void computeGravityTilt(double theta1, double theta2, double alpha,
                               double &tau1, double &tau2) {
    double t1w = theta1 + alpha;
    double t2w = theta2 + alpha;
    tau1 =
        (LINK_MASS_KG * GRAVITY * (LINK_LENGTH_M / 2.0) * cos(t1w)) +
        (LINK_MASS_KG * GRAVITY *  LINK_LENGTH_M        * cos(t1w)) +
        (END_EFFECTOR_MASS_KG * GRAVITY * LINK_LENGTH_M * cos(t1w));
    tau2 =
        (LINK_MASS_KG * GRAVITY * (LINK_LENGTH_M / 2.0) * cos(t2w)) +
        (END_EFFECTOR_MASS_KG * GRAVITY * LINK_LENGTH_M * cos(t2w));
}

void setup() {
    Serial.begin(921600);
    unsigned long t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) delay(10);

    Serial.println("Link-2 balance test starting");

    if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
        Serial.println("Failed to find BNO08x — halting");
        while (1) delay(10);
    }
    Serial.println("BNO08x found");
    setReports();

    motorL1.setup();
    motorL2.setup();
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
        float alpha_new_rad = roll_deg * DEG_TO_RAD;
        alpha_filt_rad = ROLL_LPF_BETA * alpha_new_rad +
                         (1.0f - ROLL_LPF_BETA) * alpha_filt_rad;
    }

    double sp_L1 = THETA1_OFFSET;
    double sp_L2 = -(double)alpha_filt_rad;

    double pos_L1 = initial_pos_L1 + encoder_sign_L1 * encL1.getPosition();
    double pos_L2 = initial_pos_L2 + encoder_sign_L2 * encL2.getPosition();

    double G1, G2;
    computeGravityTilt(pos_L1, pos_L2, (double)alpha_filt_rad, G1, G2);

    double effort_L1 = pidL1.calculateParallel(pos_L1, sp_L1) + torqueToDuty1 * G1;
    double effort_L2 = pidL2.calculateParallel(pos_L2, sp_L2) + torqueToDuty2 * G2;

    motorL1.drive(motor_sign_L1 * effort_L1);
    motorL2.drive(motor_sign_L2 * effort_L2);

    static unsigned long last_print = 0;
    if (millis() - last_print >= 100) {
        last_print = millis();
        Serial.printf(
            "alpha_raw=%6.2f alpha_filt=%6.2f deg | "
            "L2 sp=%6.2f cur=%6.2f deg | "
            "G2=%5.3f Nm duty2=%5.2f\n",
            roll_deg,
            alpha_filt_rad * RAD_TO_DEG,
            sp_L2 * RAD_TO_DEG,
            pos_L2 * RAD_TO_DEG,
            G2,
            motor_sign_L2 * effort_L2);
    }
}
