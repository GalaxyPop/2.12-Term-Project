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
// #define PICKUP_TRAY

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
        static unsigned long lastIncrement = 0;
        static bool forwardStarted = false;
        static bool firstForwardDone = false;
        static bool armAgainstTable = false;
        static bool forwardDone = false;
        static bool backwardDone = false;
        static bool tableResetDone = false;
        static bool gripperClosed = false;
        static bool theta2BackwardResetDone = false;
        static bool finalTheta2ResetDone = false;
        static bool finalTheta2Done = false;
        static bool homingDone = false;
        static unsigned long homingStart = 0;
        static unsigned long firstForwardStartTime = 0;
        static unsigned long secondForwardStartTime = 0;
        static unsigned long gripperCloseTime = 0;
        static unsigned long backwardStartTime = 0;

        const double INIT_THETA1 = deg2rad(36.0);
        const double INIT_THETA2 = deg2rad(-100.0);
        const double HOMING_TOL  = deg2rad(2.0);

        // ─── Homing ──────────────────────────────────────────────────
        if (!homingDone) {
            if (homingStart == 0) {
                homingStart = millis();
                targetPose.theta1 = 0.0;
                targetPose.theta2 = 0.0;
            }

            static unsigned long lastHomingStep = 0;
            const double STEP = deg2rad(5.0);
            const unsigned long STEP_INTERVAL = 200;

            if (millis() - lastHomingStep >= STEP_INTERVAL) {
                lastHomingStep = millis();

                if (targetPose.theta1 < INIT_THETA1 - deg2rad(0.5))
                    targetPose.theta1 = min(targetPose.theta1 + STEP, INIT_THETA1);
                else if (targetPose.theta1 > INIT_THETA1 + deg2rad(0.5))
                    targetPose.theta1 = max(targetPose.theta1 - STEP, INIT_THETA1);

                if (targetPose.theta2 < INIT_THETA2 - deg2rad(0.5))
                    targetPose.theta2 = min(targetPose.theta2 + STEP, INIT_THETA2);
                else if (targetPose.theta2 > INIT_THETA2 + deg2rad(0.5))
                    targetPose.theta2 = max(targetPose.theta2 - STEP, INIT_THETA2);

                Serial.printf("Homing -> theta1: %.1f° theta2: %.1f°\n",
                              targetPose.theta1 * 180.0/M_PI,
                              targetPose.theta2 * 180.0/M_PI);
            }

            updateSetpointsArmsPosition(targetPose.theta1, targetPose.theta2);
            updateSetpointsWheels(0.0, 0.0);

            double pos1 = -encoders[0].getPosition();
            double pos2 =  encoders[3].getPosition();
            bool atTarget = abs(pos1 - INIT_THETA1) < HOMING_TOL &&
                            abs(pos2 - INIT_THETA2) < HOMING_TOL;

            if (atTarget || millis() - homingStart >= 15000) {
                // Snap setpoint to current position (THETA_OFFSET after encoder
                // reset) so the subsequent filtered drift to 0 is smooth.
                encoders[0].resetPosition();
                encoders[3].resetPosition();
                resetArmPositionSetpoints();
                targetPose.theta1 = 0.0;
                targetPose.theta2 = 0.0;
                homingDone = true;
                lastIncrement = millis();
                Serial.println("=== Homing done, pickup starting ===");
            }
            return;
        }

        // ─── Phase 1-2: lift arm over the table ──────────────────────
        if (!firstForwardDone && millis() - lastIncrement >= 200) {
            lastIncrement = millis();

            if (targetPose.theta2 < deg2rad(20.0))
                targetPose.theta2 = min(targetPose.theta2 + deg2rad(6.0), deg2rad(20.0));

            if (targetPose.theta2 >= deg2rad(20.0) && targetPose.theta1 > deg2rad(-44.0))
                targetPose.theta1 = max(targetPose.theta1 - deg2rad(6.0), deg2rad(-44.0));
        }

        updateSetpointsArmsPosition(targetPose.theta1, targetPose.theta2);

        // ─── Phase 3: open gripper, drive forward a short time ───────
        if (targetPose.theta2 >= deg2rad(14.0) && !firstForwardDone) {
            gripOpen();

            if (!forwardStarted) {
                forwardStarted = true;
                firstForwardStartTime = millis();
                Serial.println("=== GRIP OPEN + FIRST FORWARD ===");
            }

            if (millis() - firstForwardStartTime < 1400) {
                double wheelVel = 0.08 / R_WHEEL;
                updateSetpointsWheels(wheelVel, wheelVel);
            } else {
                firstForwardDone = true;
                lastIncrement = millis();
                updateSetpointsWheels(0, 0);
            }
            return;
        }

        // ─── Phase 4: lower arm onto the table ───────────────────────
        if (firstForwardDone && !armAgainstTable) {
            const double TABLE_RELATIVE_THETA2 = deg2rad(-16.0);
            const double STEP = deg2rad(2.0);
            const unsigned long STEP_INTERVAL = 200;

            if (!tableResetDone) {
                encoders[3].resetPosition();
                resetArmPositionSetpoints();   // snap setpoint to measured pos
                targetPose.theta2 = 0.0;       // new relative origin for this phase
                tableResetDone = true;
                lastIncrement = millis();
            }

            if (millis() - lastIncrement >= STEP_INTERVAL) {
                lastIncrement = millis();
                if (targetPose.theta2 > TABLE_RELATIVE_THETA2 + deg2rad(0.05)) {
                    targetPose.theta2 = max(targetPose.theta2 - STEP, TABLE_RELATIVE_THETA2);
                }
            }

            updateSetpointsArmsPosition(targetPose.theta1, targetPose.theta2);
            updateSetpointsWheels(0, 0);

            if (abs(targetPose.theta2 - TABLE_RELATIVE_THETA2) < deg2rad(0.5)) {
                armAgainstTable = true;
                secondForwardStartTime = millis();
            }
            return;
        }

        // ─── Phase 5: second forward drive to push under the tray ────
        if (armAgainstTable && !forwardDone) {
            if (millis() - secondForwardStartTime < 3000) {
                double wheelVel = 0.08 / R_WHEEL;
                updateSetpointsWheels(wheelVel, wheelVel);
            } else {
                forwardDone = true;
                updateSetpointsWheels(0, 0);
            }
            updateSetpointsArmsPosition(targetPose.theta1, targetPose.theta2);
        }

        // ─── Phase 6: close gripper ──────────────────────────────────
        if (forwardDone && !gripperClosed) {
            gripClose();
            gripperClosed = true;
            gripperCloseTime = millis();
            updateSetpointsArmsPosition(targetPose.theta1, targetPose.theta2);
            updateSetpointsWheels(0, 0);
            return;
        }

        // ─── Phase 7: back up while lifting arm ──────────────────────
        if (gripperClosed && !backwardDone) {
            const double THETA1_BACKWARD_TARGET = deg2rad(45.0);
            const double THETA2_BACKWARD_TARGET = deg2rad(10.0);
            const double STEP_THETA1 = deg2rad(5.0);
            const double STEP_THETA2 = deg2rad(4.0);
            const unsigned long STEP_INTERVAL = 200;

            // Dwell 1 s after closing the gripper so the grip settles.
            if (millis() - gripperCloseTime < 1000) {
                updateSetpointsArmsPosition(targetPose.theta1, targetPose.theta2);
                updateSetpointsWheels(0, 0);
                return;
            }

            if (!theta2BackwardResetDone) {
                encoders[0].resetPosition();
                encoders[3].resetPosition();
                resetArmPositionSetpoints();
                targetPose.theta1 = 0.0;
                targetPose.theta2 = 0.0;
                theta2BackwardResetDone = true;
                lastIncrement = millis();
                backwardStartTime = millis();
                Serial.println("=== Arm reset, backward + lift starting ===");
            }

            if (millis() - lastIncrement >= STEP_INTERVAL) {
                lastIncrement = millis();

                if (targetPose.theta1 < THETA1_BACKWARD_TARGET - deg2rad(0.5)) {
                    targetPose.theta1 = min(targetPose.theta1 + STEP_THETA1, THETA1_BACKWARD_TARGET);
                }
                if (targetPose.theta2 < THETA2_BACKWARD_TARGET - deg2rad(0.5)) {
                    targetPose.theta2 = min(targetPose.theta2 + STEP_THETA2, THETA2_BACKWARD_TARGET);
                }

                Serial.printf("Backward arm -> theta1: %.1f deg | theta2: %.1f deg\n",
                              targetPose.theta1 * 180.0 / M_PI,
                              targetPose.theta2 * 180.0 / M_PI);
            }

            if (millis() - backwardStartTime < 7000) {
                double wheelVel = -0.10 / R_WHEEL;
                updateSetpointsWheels(wheelVel, wheelVel);
            } else {
                backwardDone = true;
                updateSetpointsWheels(0, 0);
                Serial.println("=== Backward done, final hold ===");
            }
            updateSetpointsArmsPosition(targetPose.theta1, targetPose.theta2);
            return;
        }

        // ─── Phase 8: drop arm back down to stow ─────────────────────
        if (backwardDone && !finalTheta2Done) {
            const double FINAL_THETA2_TARGET = deg2rad(-10.0);
            const double STEP_THETA2 = deg2rad(2.0);
            const unsigned long STEP_INTERVAL = 200;

            if (!finalTheta2ResetDone) {
                encoders[3].resetPosition();
                resetArmPositionSetpoints();
                targetPose.theta2 = 0.0;
                finalTheta2ResetDone = true;
                lastIncrement = millis();
            }

            if (millis() - lastIncrement >= STEP_INTERVAL) {
                lastIncrement = millis();
                if (targetPose.theta2 > FINAL_THETA2_TARGET + deg2rad(0.5)) {
                    targetPose.theta2 = max(targetPose.theta2 - STEP_THETA2, FINAL_THETA2_TARGET);
                }
            }

            updateSetpointsArmsPosition(targetPose.theta1, targetPose.theta2);
            updateSetpointsWheels(0, 0);

            if (abs(targetPose.theta2 - FINAL_THETA2_TARGET) < deg2rad(0.5)) {
                finalTheta2Done = true;
            }
            return;
        }

        // ─── Final hold ──────────────────────────────────────────────
        updateSetpointsArmsPosition(targetPose.theta1, targetPose.theta2);
        updateSetpointsWheels(0, 0);
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
