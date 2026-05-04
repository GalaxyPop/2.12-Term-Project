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
#include "jetson_link.h"

// #define AUTONOMOUS

extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;
bool joystickOverride = false; // once joystick data is received, manual control owns the robot until reset

// Joystick deadzone (above this in |x| or |y|, joystick takes priority).
static constexpr double JOY_DEADZONE = 0.1;
// Watchdog: stale Jetson vel command after this many ms -> stop.
static constexpr uint32_t JETSON_VEL_TIMEOUT_MS = 200;

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
bool servo_open = 0;

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
    bool joy_touched = freshWirelessData &&
        (fabs(controllerMessage.joystick1.x) > JOY_DEADZONE ||
         fabs(controllerMessage.joystick1.y) > JOY_DEADZONE);

    if (joy_touched) {
        joystickOverride = true;
        freshWirelessData = false;
        double forward = mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_FORWARD, MAX_FORWARD);
        double turn    = mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_TURN, MAX_TURN);

        double x_pos = mapStick(controllerMessage.joystick2.x, MAX_DIST);
        double y_pos = mapStick(controllerMessage.joystick2.y, MAX_DIST);
        // double x_pos = mapStick(controllerMessage.joystick2.x, 1);
        // double y_pos = mapStick(controllerMessage.joystick2.y, 1);

        targetXY = {x_pos, y_pos};
        targetXY = createBarrier(targetXY); // prevents the arm from colliding with the robot body
        targetPose = inverseKinematics(targetXY);

        updateSetpointsWheels(forward + turn, forward - turn);
        updateSetpointsArms(targetPose.theta1, targetPose.theta2);
        // updateSetpointsArms(x_pos, y_pos); // velocity control
        return;

        
        // Serial.printf("Servo", controllerMessage.buttonR);
        //SERVO TEST CODE:
        if (controllerMessage.buttonR && !servo_open) {
                gripOpen();
                servo_open = true;
                Serial.println("Servo Open");
        } else if (!controllerMessage.buttonR && servo_open) {
                gripClose();
                servo_open = false;
                Serial.println("Servo Closed");
        }
    

    }

    // Consume any stale untouched packet so it doesn't linger and block Jetson.
    if (freshWirelessData) freshWirelessData = false;

    if (!g_jetson_estop &&
        g_jetson_vel_ts_ms != 0 &&
        (millis() - g_jetson_vel_ts_ms) < JETSON_VEL_TIMEOUT_MS) {
        v_omega_to_wheels(g_jetson_vel_v, g_jetson_vel_w);
        return;


    if (controllerMessage.buttonR && !servo_open) {
        gripOpen();
        servo_open = true;
    } else if (controllerMessage.buttonR && servo_open) {
        gripClose();
        servo_open = false;
    }
}

    #ifdef AUTONOMOUS
    if (!joystickOverride) {
        runAutonomousSequence();
        return;
    }
    #endif

    // Watchdog: nothing fresh from either source.
    updateSetpointsWheels(0.0, 0.0);
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
