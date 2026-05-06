#include <Arduino.h>
#include <math.h>
#include "util.h"
#include "robot_drive.h"
#include "EncoderVelocity.h"
#include "wireless.h"
#include "robot_motion_control.h"
#include "kinematics.h"
#include "trajectories.h"
#include "robot_imu.h"   // ypr.roll for tray-balancing

// #define UTURN
// #define CIRCLE
//#define JOYSTICK
// #define YOUR_TRAJECTORY
// #define VERTICAL_LINE
#define RAMP
// #define TEST_ARM

extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;

// armPositionControl is defined in robot_motion_control.cpp. The ramp_test
// env builds this file in place of that one, so we provide a definition here.
// Always true for ramp traversal — arm is driven by position setpoint.
bool armPositionControl = true;

// ---------- Tray-balance constants (GUESSES — VERIFY ON BENCH AND TWEAK) -----
// The robot has a passive platform that the held tray rests on, so link 2
// doesn't need to be exactly horizontal — a slight downward tilt keeps the
// tray pinned. Both angles are eyeball estimates; test on the bench and
// adjust before any scored run.
//   - BALANCE_THETA1_DEG       : link 1 angle in CHASSIS frame.
//   - BALANCE_THETA2_DEG_WORLD : link 2 angle in WORLD frame. Chassis-frame
//     setpoint becomes (world value) - alpha so the world-frame tilt stays
//     constant as the chassis pitches up the ramp.
static constexpr double BALANCE_THETA1_DEG       = 110.0;   // GUESS
static constexpr double BALANCE_THETA2_DEG_WORLD = 0 ; //GUESS

// Single-pole IIR on chassis pitch (~50 ms tau at 200 Hz IMU). Mirrors
// test_balance_link2.cpp — absorbs the ~1 deg sensor jitter on ypr.roll.
static constexpr double ROLL_LPF_BETA = 0.1;
static double alpha_filt_rad = 0.0;

// based off of initial position of robot arm being straight up, so theta1 is 90 deg and theta2 is 0 deg
JointSpace targetPose = {THETA1_OFFSET, 0.0}; //initial setpoint
TaskSpace targetXY = {0, L1 + L2}; //initial position of end effector
TaskSpace nominalPosition = {0.5*(L1 + L2), 0}; //nominal position of end effector for trajectories

int state = 0;
double robotVelocity = 0; // velocity of robot, in m/s
double k = 0; // curvature k is 1/radius from center of rotation circle

extern EncoderVelocity encoders[NUM_MOTORS];
double currPhiL = 0;
double currPhiR = 0;
double prevPhiL = 0;
double prevPhiR = 0;
double t1 = 0;
double t2 = 0;

double rampStartX = 0;
double rampStartY = 0;
bool rampInitialized = false;
unsigned long lastRampPrint = 0;
double rampStartTheta = 0;

// Sets the desired wheel velocities based on desired robot velocity in m/s
// and k curvature in 1/m representing 1/(radius of curvature). Wheels only —
// arm setpoints are managed by the tray-balance block in followTrajectory().
void setWheelVelocities(float robotVelocity, float k){
    double left = (robotVelocity - k * B_BASE * robotVelocity) / R_WHEEL;
    double right = 2 * robotVelocity / R_WHEEL  - left;
    updateSetpointsWheels(left, right);
}

// Makes robot body follow a trajectory
void followTrajectory() {

    #ifdef JOYSTICK
    if (freshWirelessData) {
        freshWirelessData = false;
        double forward = abs(controllerMessage.joystick1.y) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_FORWARD, MAX_FORWARD);
        double turn = abs(controllerMessage.joystick1.x) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_TURN, MAX_TURN);

        // double x_pos = mapStick(controllerMessage.joystick2.x, MIN_DIST, MAX_DIST);
        // double y_pos = mapStick(controllerMessage.joystick2.y, MIN_DIST, MAX_DIST);

        // targetXY = {x_pos, y_pos};
        // // targetXY = getClosestPointInWorkspace(targetXY); // constains input x and y to workspace of robot arm
        // targetPose = inverseKinematics(targetXY);

        // updateSetpoints(forward + turn, forward - turn, targetPose.theta1, targetPose.theta2); // theta1 and theta2 being for the arm links

        double arm_x = controllerMessage.joystick2.x;
        double arm_y = controllerMessage.joystick2.y;

        // Only update the arm target when joystick 2 is actually moved.
        // When joystick 2 is released, keep the previous targetPose.
        if (abs(arm_x) > 0.1 || abs(arm_y) > 0.1) {
            double x_pos = mapStick(arm_x, MIN_DIST, MAX_DIST);
            double y_pos = mapStick(arm_y, MIN_DIST, MAX_DIST);

            targetXY = {x_pos, y_pos};
            //targetXY = getClosestPointInWorkspace(targetXY);
            targetPose = inverseKinematics(targetXY);
        }

        updateSetpoints(forward + turn, forward - turn, targetPose.theta1, targetPose.theta2);

        // Serial.print("theta1: ");
        // Serial.print(targetPose.theta1);
        // Serial.print(" | theta2: ");
        // Serial.println(targetPose.theta2);
    }
    #endif

    #ifdef RAMP

    if (!rampInitialized) {
        rampStartX = robotMessage.x;
        rampStartY = robotMessage.y;
        rampStartTheta = robotMessage.theta;
        rampInitialized = true;
    }

    double dx = robotMessage.x - rampStartX;
    double dy = robotMessage.y - rampStartY;
    double dist = sqrt(dx * dx + dy * dy);

    // ---- Tray balancing ----
    // ypr.roll is sign-flipped in robot_imu.cpp so +ve = climbing. The LPF
    // smooths out ~1 deg jitter on the raw reading. theta1 is held in the
    // chassis frame; theta2 compensates for chassis pitch so its world-frame
    // angle stays at BALANCE_THETA2_DEG_WORLD as the bot climbs.
    double alpha_raw_rad = ypr.roll * DEG_TO_RAD;
    alpha_filt_rad = ROLL_LPF_BETA * alpha_raw_rad +
                     (1.0 - ROLL_LPF_BETA) * alpha_filt_rad;
    // Push filtered pitch into the global gravity comp so torques on both
    // links use the world-frame moment arm during the climb.
    setChassisPitch(alpha_filt_rad);
    targetPose.theta1 =  BALANCE_THETA1_DEG       * M_PI / 180.0;
    targetPose.theta2 = (BALANCE_THETA2_DEG_WORLD * M_PI / 180.0) - alpha_filt_rad;

    // 2 sec arming delay from the first followTrajectory() call — gives the
    // arm time to slew to the balance pose and lets you get hands clear
    // before the wheels start. Arm setpoints still get pushed below.
    static unsigned long armingStart = 0;
    if (armingStart == 0) armingStart = millis();
    if (millis() - armingStart < 2000) {
        updateSetpointsWheels(0.0, 0.0);
        updateSetpointsArmsPosition(targetPose.theta1, targetPose.theta2);
        return;
    }

    switch (state) {

            case 0:
                // Climb ramp
                robotVelocity = 0.90; //don't touch this value, it's the best speed for climbing the ramp without slipping
                k = 0;

                if (dist >= 0.38) { // distance to climb the ramp, star is when the back wheels are at the same level avec the wheel of the table
                    state++;
                    rampStartX = robotMessage.x;
                    rampStartY = robotMessage.y;
                }
                break;

            case 1:
                // Move on platform
                robotVelocity = 0.15;
                k = 0;

                if (dist >= 0.10) {
                    state++;
                    rampStartTheta = robotMessage.theta;
                }
                break;

            case 2: {
                // Turn in place for a fixed duration
                static unsigned long turnStartTime = 0;

                if (turnStartTime == 0) {
                    turnStartTime = millis();
                }

                updateSetpointsWheels(-4, 4);
                // Arm targets are pushed below the switch via the
                // tray-balance updateSetpointsArmsPosition() call.

                if (millis() - turnStartTime >= 3900) {  // don't touch the time for now (3750)
                    state++;
                    turnStartTime = 0;

                    rampStartX = robotMessage.x;
                    rampStartY = robotMessage.y;
                }

                break;
            }

            case 3:
                // Descend ramp
                robotVelocity = - 0.18;
                k = 0;

                if (dist >= 0.38) {
                    state++;
                }
                break;
        }   // close switch (was missing)

        // && fixes a pre-existing typo: original `or` made this tautological,
        // which would clobber state 2's turn-in-place wheel command.
        if (state != 2 && state != 4) {
            setWheelVelocities(robotVelocity, k);
        }

        // Push tray-balance arm targets every loop, regardless of state.
        updateSetpointsArmsPosition(targetPose.theta1, targetPose.theta2);

    #endif

    #ifdef TEST_ARM

    static bool initialized = false;
    static double t1;
    static double t2;

    double target1 = 135.0 * M_PI / 180.0;
    double target2 = -110.0 * M_PI / 180.0;

    if (!initialized) {
        // Start from actual measured arm position at reset
        t1 = positions[0];  // joint 1
        t2 = positions[3];  // joint 2
        initialized = true;
    }

    double step = 0.001;  // rad per loop, slow

    if (abs(target1 - t1) > step) {
        t1 += (target1 > t1) ? step : -step;
    } else {
        t1 = target1;
    }

    if (abs(target2 - t2) > step) {
        t2 += (target2 > t2) ? step : -step;
    } else {
        t2 = target2;
    }

    updateSetpoints(0, 0, t1, t2);

    #endif

  
    #ifdef CIRCLE
    robotVelocity = 0.2;
    k = 1/0.5;
    setWheelVelocities(robotVelocity, k);
    #endif

    #ifdef UTURN
    switch (state) {
        case 0:
            // Until robot has achieved an x translation of 1 m:
            if (robotMessage.x <= 1.0) {
                // Move in a straight line forward
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;

        case 1:
            // Until robot has achieved a 180 deg turn in theta:
            if (robotMessage.theta <= M_PI) {
                // Turn in a circle with radius 25 cm
                robotVelocity = 0.2;
                k = 1 / 0.25;
            } else {
                state++;
            }
            break;

        case 2:
            // Until robot has achieved an x translation of -1 m:
            if (robotMessage.x >= 0) {
                // Move in a straight line forward
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;

        default:
            // If not in any of the states, robot should just stop
            robotVelocity = 0;
            k = 0;
            break;
    }
    setWheelVelocities(robotVelocity, k);
    #endif

    #ifdef YOUR_TRAJECTORY
    switch (state) {
        case 0:
            // Until robot has achieved an x translation of 0.5 m:
            if (robotMessage.x <= 0.5) {
                // Move in a straight line forward
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;

        case 1:
            // Until robot has achieved a 90 deg turn in theta:
            if (robotMessage.theta <= M_PI/2) {
                // Turn in a circle with radius 25 cm
                robotVelocity = 0.2;
                k = 1 / 0.25;
            } else {
                state++;
            }
            break;

        case 2:
            // Until robot has backed up a 90 deg turn in theta:
            if (robotMessage.theta <= M_PI) {
                // Turn in a circle with radius 25 cm
                robotVelocity = -0.2;
                k = -1 / 0.25;
            } else {
                state++;
            }
            break;

        case 3:
            // Until robot has achieved an x translation of -0.5 m:
            if (robotMessage.x >= 0) {
                // Move in a straight line forward
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;

        default:
            // If not in any of the states, robot should just stop
            robotVelocity = 0;
            k = 0;
            break;
    }
    setWheelVelocities(robotVelocity, k);
    #endif

    // control arm joints to do vertical line
    #ifdef VERTICAL_LINE
        double amplitude = 5; // amplitude of vertical line in cm
        double frequency = 0.5; // frequency of vertical line in Hz
        double time = millis(); // time in ms

        targetXY.x = nominalPosition.x;
        targetXY.y = nominalPosition.y + amplitude*sin(2*M_PI*frequency*time/1000.0);
        targetXY = getClosestPointInWorkspace(targetXY);

        targetPose = inverseKinematics(targetXY);
        updateSetpoints(0, 0, targetPose.theta1, targetPose.theta2);
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


