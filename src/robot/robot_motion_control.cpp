#include <Arduino.h>
#include "util.h"
#include "robot_drive.h"
#include "EncoderVelocity.h"
#include "wireless.h"
#include "robot_motion_control.h"
#include "kinematics.h"
#include "trajectories.h"
#include "robot_autonomous.h"

#define JOYSTICK

extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;

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

// Sets the desired wheel velocities based on desired robot velocity in m/s
// and k curvature in 1/m representing 1/(radius of curvature)
void setWheelVelocities(float robotVelocity, float k){
    double left = (robotVelocity - k * bb * robotVelocity) / r;
    double right = 2 * robotVelocity / r  - left;
    updateSetpointsWheels(left, right);
}

// If the robot is reading joystick data, it will follow the joystick input.
// Otherwise it will run the autonomous sequence.
void followTrajectory() {
    #ifdef JOYSTICK
    if (freshWirelessData) {
        freshWirelessData = false;
        double forward = abs(controllerMessage.joystick1.y) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_FORWARD, MAX_FORWARD);
        double turn = abs(controllerMessage.joystick1.x) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_TURN, MAX_TURN);

        double x_pos = mapStick(controllerMessage.joystick2.x, MAX_DIST);
        double y_pos = mapStick(controllerMessage.joystick2.y, MAX_DIST);

        targetXY = {x_pos, y_pos};
        targetXY = getClosestPointInWorkspace(targetXY); // constains input x and y to workspace of robot arm
        targetXY = createBarrier(targetXY); // prevents the arm from colliding with the robot body
        targetPose = inverseKinematics(targetXY);

        updateSetpointsWheels(forward + turn, forward - turn); // left and right wheel velocities
        updateSetpointsArms(targetPose.theta1, targetPose.theta2);
    } else {
        runAutonomousSequence();
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
    float dtheta = r / (2 * bb) * (dPhiR - dPhiL);
    float dx = r / 2.0 * (cos(robotMessage.theta) * dPhiR + cos(robotMessage.theta) * dPhiL);
    float dy = r / 2.0 * (sin(robotMessage.theta) * dPhiR + sin(robotMessage.theta) * dPhiL);

    // Update robot message
    robotMessage.millis = millis();
    robotMessage.x += dx;
    robotMessage.y += dy;
    robotMessage.theta += dtheta;
}
