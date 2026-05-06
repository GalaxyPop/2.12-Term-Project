#include <Arduino.h>
#include <math.h>
#include "util.h"
#include "robot_drive.h"
#include "EncoderVelocity.h"
#include "wireless.h"
#include "robot_motion_control.h"
#include "kinematics.h"
#include "trajectories.h"
#include "robot_autonomous.h"
#include "servo_control.h"
#include "robot_imu.h"
// #include "jetson_link.h"

// #define AUTONOMOUS
// #define RAMP
#define PICKUP_TRAY

#if defined(RAMP) && defined(PICKUP_TRAY)
    #error "Define only one of RAMP or PICKUP_TRAY at a time."
#endif

#if defined(RAMP) || defined(PICKUP_TRAY)
    #define FSM_OWNS_MOTION
#endif

extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;
bool joystickOverride = false; // once joystick data is received, manual control owns the robot until reset
extern euler_t ypr;

bool armPositionControl = false; // if true, arm's position will be controlled by joystick inputs
bool prevArmPositionControl = armPositionControl;
bool endEffectorControl = false; // if true, gripper will be closed, otherwise it will be open
bool prevEndEffectorControl = endEffectorControl;

// Jointspace and taskspace representations of the arm's target position.
JointSpace targetPose = {THETA1_OFFSET, 0.0};
TaskSpace targetXY = {0, L1 + L2};

// Watchdog: stale Jetson vel command after this many ms -> stop.
static constexpr uint32_t JETSON_VEL_TIMEOUT_MS = 200;

double robotVelocity = 0; // velocity of robot, in m/s
double k = 0; // curvature k is 1/radius from center of rotation circle

extern EncoderVelocity encoders[NUM_MOTORS];
extern double positions[NUM_MOTORS];  // absolute joint angles, maintained by updateArms()
double currPhiL = 0;
double currPhiR = 0;
double prevPhiL = 0;
double prevPhiR = 0;
double t1 = 0;
double t2 = 0;

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// Sets the desired wheel velocities based on desired robot velocity in m/s
// and k curvature in 1/m representing 1/(radius of curvature)
void setWheelVelocities(float robotVelocity, float k){
    double left = (robotVelocity - k * B_BASE * robotVelocity) / R_WHEEL;
    double right = 2 * robotVelocity / R_WHEEL  - left;
    updateSetpointsWheels(left, right);
}

// Priority (highest first):
//   1. Joystick on touch: a fresh ESP-NOW packet with |joystick1| above the
//      deadzone means the operator has grabbed control -> use joystick,
//      ignore Jetson. Releasing the stick hands control back within one tick.
//   2. Jetson vel cmd: last cmd < 200 ms old and not estopped -> use (v, w).
//   3. AUTONOMOUS mode (compile-time, non-default): fall through to sequencer.
//   4. Watchdog: stop wheels.
//
// `joystickOverride` stays latched once touched (used by AUTONOMOUS mode to
// prevent the scripted sequence from fighting the operator), but does NOT
// block Jetson control — serial commands can resume the moment the stick is
// released.

void followTrajectory() {
#ifdef FSM_OWNS_MOTION
    // RAMP / PICKUP_TRAY own motion exclusively — drop any joystick packets
    // so the operator can't latch control mid-sequence.
    if (freshWirelessData) freshWirelessData = false;
#else
    if (freshWirelessData) { // if contoller is sending information to robot
        freshWirelessData = false;
        joystickOverride = true; // stops autonomous sequence and switches to joystick control permanently

        armPositionControl = controllerMessage.buttonL; // if left button is pressed, joystick controls arm position instead of velocity
        if (armPositionControl != prevArmPositionControl) {
            if (armPositionControl) {
                resetArmPositionSetpoints();
            } else {
                updateSetpointsArmsVelocity(0, 0);
            }
            prevArmPositionControl = armPositionControl;
        }

        // end effector control (gripper open/close)
        endEffectorControl = controllerMessage.buttonR; // if right button is pressed, gripper closes, otherwise it opens
        if (endEffectorControl != prevEndEffectorControl) {
            if (endEffectorControl) {
                gripClose();
            } else {
                gripOpen();
            }
            prevEndEffectorControl = endEffectorControl;
        }

        // wheel control
        double forward = abs(controllerMessage.joystick1.y) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_FORWARD, MAX_FORWARD);
        double turn = abs(controllerMessage.joystick1.x) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_TURN, MAX_TURN);
        updateSetpointsWheels(forward + turn, forward - turn); // left and right wheel velocities

        // arm control
        if (armPositionControl) { // joystick controls arm position instead of velocity
            double x_pos = mapStick(controllerMessage.joystick2.x, MAX_DIST);
            double y_pos = mapStick(controllerMessage.joystick2.y, MAX_DIST);

            targetXY = {x_pos, y_pos};
            targetXY = createBarrier(targetXY); // prevents the arm from colliding with the robot body
            targetPose = inverseKinematics(targetXY);
            updateSetpointsArmsPosition(targetPose.theta1, targetPose.theta2);

            // IMU balancing:
            // updateSetpointsArms(PI/2, -ypr.roll);

            // IMU testing
            updateSetpointsArmsPosition(M_PI/2, 0);

        } else { // joystick controls arm velocity instead of position
            double theta1_dot = mapStick(controllerMessage.joystick2.x, MAX_SPEED);
            double theta2_dot = mapStick(controllerMessage.joystick2.y, MAX_SPEED);
            updateSetpointsArmsVelocity(theta1_dot, theta2_dot);
        }
    }
#endif

    #ifdef RAMP
    {
        // Distance-gated ramp climb: drive up, cross platform, turn in place.
        // Arms are held by whatever updatePIDs(false) does (velocity 0) — this
        // block does not touch arm setpoints.
        static int rampState = 0;
        static double rampStartX = 0;
        static double rampStartY = 0;
        static bool rampInitialized = false;

        if (!rampInitialized) {
            rampStartX = robotMessage.x;
            rampStartY = robotMessage.y;
            rampInitialized = true;
        }

        double dx = robotMessage.x - rampStartX;
        double dy = robotMessage.y - rampStartY;
        double dist = sqrt(dx * dx + dy * dy);

        switch (rampState) {
            case 0:
                // Climb ramp — tuned so the back wheels don't slip at ramp start.
                robotVelocity = 0.4;
                k = 0;
                if (dist >= 0.33) {
                    rampState++;
                    rampStartX = robotMessage.x;
                    rampStartY = robotMessage.y;
                }
                break;

            case 1:
                // Move on platform.
                robotVelocity = 0.2;
                k = 0;
                if (dist >= 0.10) {
                    rampState++;
                }
                break;

            case 2: {
                // Turn in place for a fixed duration.
                static unsigned long turnStartTime = 0;
                if (turnStartTime == 0) turnStartTime = millis();

                updateSetpointsWheels(-4, 4);

                if (millis() - turnStartTime >= 4000) {
                    rampState++;
                    turnStartTime = 0;
                    rampStartX = robotMessage.x;
                    rampStartY = robotMessage.y;
                }
                break;
            }

            default:
                robotVelocity = 0;
                k = 0;
                break;
        }

        // Case 2 drives the wheels directly; don't overwrite its setpoints.
        if (rampState != 2) {
            setWheelVelocities(robotVelocity, k);
        }
        return;
    }
    #endif

    #ifdef PICKUP_TRAY
    {
        // PICKUP_TRAY commands arm joints via positionSetpoints[], so the
        // PIDs must run in position mode. The global is otherwise only
        // flipped by the joystick path, which FSM_OWNS_MOTION disables.
        armPositionControl = true;

        // ─── Trigger gate ────────────────────────────────────────────
        // Sequence stays IDLE until an operator trigger arrives. Two
        // sources, OR'd:
        //   1. Serial 'g' byte (current testing). Works from any host
        //      that opens /dev/ttyACM0 at 921600 — pio monitor window,
        //      screen, or a python serial.write from an SSH session
        //      on the Jetson.
        //   2. D-pad UP (placeholder). controllerMessage.dPad is not
        //      populated by the current controller firmware, so this
        //      branch is inert today; it activates automatically once
        //      the controller populates dPad.up.
        //
        // TODO: remove the serial branch when handleJetsonSerial() is
        //       enabled in robot_main.cpp — they share Serial.read().
        static bool pickupArmed = false;
        static bool idleAnnounced = false;
        static bool prevDpad = false;

        if (!pickupArmed) {
            if (!idleAnnounced) {
                Serial.println("=== PICKUP_TRAY idle — send 'g' over serial or press D-pad UP ===");
                idleAnnounced = true;
            }

            while (Serial.available()) {
                int c = Serial.read();
                if (c == 'g' || c == 'G') pickupArmed = true;
            }

            bool dpad = controllerMessage.dPad.up;
            if (dpad && !prevDpad) pickupArmed = true;
            prevDpad = dpad;

            if (pickupArmed) Serial.println("=== PICKUP_TRAY armed — sequence starting ===");

            updateSetpointsWheels(0, 0);
            return;
        }

        // ─── Absolute-frame joint targets ────────────────────────────
        // All angles in positions[] frame (rad). Arm is vertical-up at
        // boot: positions[0] = positions[3] = π/2.
        //
        // Sign convention assumption: decreasing absolute angle rotates
        // the joint toward the chassis/forward (inferred empirically
        // from the earlier frame-mismatched run where setpoint π/2→0
        // swung the arm into the chassis). If hardware shows motion in
        // the wrong direction, flip the sign of the offset in the
        // relevant constant (e.g., M_PI/2 - X  ↔  M_PI/2 + X).
        const double HOME_J1  = M_PI/2 - deg2rad(36.0);   // ≈  54°
        const double HOME_J2  = M_PI/2 - deg2rad(100.0);  // ≈ -10°
        const double LIFT_J1  = M_PI/2 - deg2rad(80.0);   // homed j1 - 44° lift
        const double LIFT_J2  = M_PI/2 - deg2rad(80.0);   // homed j2 + 20° lift
        const double TABLE_J1 = LIFT_J1;                  // hold j1
        const double TABLE_J2 = LIFT_J2 - deg2rad(16.0);  // lower j2 onto table
        const double LIFT2_J1 = LIFT_J1 + deg2rad(45.0);  // rise for reverse
        const double LIFT2_J2 = TABLE_J2 + deg2rad(10.0); // rise for reverse
        const double STOW_J1  = LIFT2_J1;                 // hold j1
        const double STOW_J2  = LIFT2_J2 - deg2rad(10.0); // final droop

        const double POSE_TOL = deg2rad(3.0);

        enum Phase {
            PH_HOME = 0, PH_LIFT, PH_FWD1, PH_TABLE, PH_FWD2,
            PH_GRIP, PH_BACK, PH_STOW, PH_HOLD
        };
        static int phase = PH_HOME;
        static unsigned long phaseStart = 0;
        static bool phaseAnnounced = false;

        auto enterPhase = [&](int next, const char *name) {
            phase = next;
            phaseStart = millis();
            phaseAnnounced = false;
            Serial.printf("=== Phase %d: %s ===\n", next, name);
        };

        auto atPose = [&](double j1, double j2) {
            return fabs(positions[0] - j1) < POSE_TOL &&
                   fabs(positions[3] - j2) < POSE_TOL;
        };

        // Initialize phaseStart on first armed tick.
        if (phaseStart == 0) phaseStart = millis();

        switch (phase) {
            case PH_HOME: {
                updateSetpointsArmsPosition(HOME_J1, HOME_J2);
                updateSetpointsWheels(0, 0);
                if (atPose(HOME_J1, HOME_J2) || millis() - phaseStart >= 8000) {
                    enterPhase(PH_LIFT, "LIFT");
                }
                break;
            }
            case PH_LIFT: {
                updateSetpointsArmsPosition(LIFT_J1, LIFT_J2);
                updateSetpointsWheels(0, 0);
                if (atPose(LIFT_J1, LIFT_J2) || millis() - phaseStart >= 5000) {
                    gripOpen();
                    enterPhase(PH_FWD1, "FWD1 + grip open");
                }
                break;
            }
            case PH_FWD1: {
                updateSetpointsArmsPosition(LIFT_J1, LIFT_J2);
                double wv = 0.08 / R_WHEEL;
                updateSetpointsWheels(wv, wv);
                if (millis() - phaseStart >= 1400) {
                    updateSetpointsWheels(0, 0);
                    enterPhase(PH_TABLE, "lower to table");
                }
                break;
            }
            case PH_TABLE: {
                updateSetpointsArmsPosition(TABLE_J1, TABLE_J2);
                updateSetpointsWheels(0, 0);
                if (atPose(TABLE_J1, TABLE_J2) || millis() - phaseStart >= 4000) {
                    enterPhase(PH_FWD2, "FWD2 under tray");
                }
                break;
            }
            case PH_FWD2: {
                updateSetpointsArmsPosition(TABLE_J1, TABLE_J2);
                double wv = 0.08 / R_WHEEL;
                updateSetpointsWheels(wv, wv);
                if (millis() - phaseStart >= 3000) {
                    updateSetpointsWheels(0, 0);
                    gripClose();
                    enterPhase(PH_GRIP, "grip close + dwell");
                }
                break;
            }
            case PH_GRIP: {
                updateSetpointsArmsPosition(TABLE_J1, TABLE_J2);
                updateSetpointsWheels(0, 0);
                if (millis() - phaseStart >= 1000) {
                    enterPhase(PH_BACK, "back up + lift");
                }
                break;
            }
            case PH_BACK: {
                updateSetpointsArmsPosition(LIFT2_J1, LIFT2_J2);
                double wv = -0.10 / R_WHEEL;
                updateSetpointsWheels(wv, wv);
                if (millis() - phaseStart >= 7000) {
                    updateSetpointsWheels(0, 0);
                    enterPhase(PH_STOW, "stow arm");
                }
                break;
            }
            case PH_STOW: {
                updateSetpointsArmsPosition(STOW_J1, STOW_J2);
                updateSetpointsWheels(0, 0);
                if (atPose(STOW_J1, STOW_J2) || millis() - phaseStart >= 4000) {
                    enterPhase(PH_HOLD, "hold");
                }
                break;
            }
            default: { // PH_HOLD
                updateSetpointsArmsPosition(STOW_J1, STOW_J2);
                updateSetpointsWheels(0, 0);
                break;
            }
        }

        // Log phase state once on entry and every ~1s thereafter.
        static unsigned long lastPhaseLog = 0;
        if (!phaseAnnounced || millis() - lastPhaseLog >= 1000) {
            phaseAnnounced = true;
            lastPhaseLog = millis();
            Serial.printf("  phase=%d  j1=%.1f° j2=%.1f°\n",
                          phase,
                          positions[0] * 180.0 / M_PI,
                          positions[3] * 180.0 / M_PI);
        }

        return;
    }
    #endif

    // if (!g_jetson_estop &&
    //     g_jetson_vel_ts_ms != 0 &&
    //     (millis() - g_jetson_vel_ts_ms) < JETSON_VEL_TIMEOUT_MS) {
    //     v_omega_to_wheels(g_jetson_vel_v, g_jetson_vel_w);
    //     return;
    // }

    #ifdef AUTONOMOUS
        if (!joystickOverride) {
            runAutonomousSequence();
            return;
        }
    #endif
}

void updateOdometry() {
    // Take angles from traction (rear) wheels only since they don't slip
    currPhiL = encoders[2].getPosition();
    currPhiR = -encoders[3].getPosition();

    // Update wheel angles and angular change
    double dPhiL = currPhiL - prevPhiL;
    double dPhiR = currPhiR - prevPhiR;
    prevPhiL = currPhiL;
    prevPhiR = currPhiR;

    // Calculate update in robot's base coordinates
    float dtheta = R_WHEEL / (2 * B_BASE) * (dPhiR - dPhiL);
    float dx = R_WHEEL / 2.0 * (cos(robotMessage.theta) * dPhiR + cos(robotMessage.theta) * dPhiL);
    float dy = R_WHEEL / 2.0 * (sin(robotMessage.theta) * dPhiR + sin(robotMessage.theta) * dPhiL);

    // Update robot message
    robotMessage.millis = millis();
    robotMessage.x += dx;
    robotMessage.y += dy;
    robotMessage.theta += dtheta;
}
