#include <Arduino.h>
#include "util.h"
#include "robot_drive.h"
#include "EncoderVelocity.h"
#include "wireless.h"
#include "robot_motion_control.h"
#include "kinematics.h"
#include "trajectories.h"

// #define UTURN
// #define CIRCLE
#define JOYSTICK
// #define YOUR_TRAJECTORY
// #define VERTICAL_LINE

extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;

// based off of initial position of robot arm being straight up, so theta1 is 90 deg and theta2 is 0 deg
JointSpace targetPose = {THETA1_OFFSET, THETA2_OFFSET}; //initial setpoint
TaskSpace targetXY = forwardKinematics(targetPose); //initial position of end effector
TaskSpace nominalPosition = {0.5*(L1 + L2), 0}; //nominal position of end effector for custom trajectories

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
    updateSetpoints(left, right, 0, 0);
}

// Makes robot body follow a trajectory
void followTrajectory() {

    #ifdef JOYSTICK
    if (freshWirelessData) {
        freshWirelessData = false;
        double forward = abs(controllerMessage.joystick1.y) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_FORWARD, MAX_FORWARD);
        double turn = abs(controllerMessage.joystick1.x) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_TURN, MAX_TURN);

        double x_pos = abs(controllerMessage.joystick2.x) < 0.1 ? 0 : mapDouble(controllerMessage.joystick2.x, -1, 1, -MAX_DIST, MAX_DIST);
        double y_pos = abs(controllerMessage.joystick2.y) < 0.1 ? 0 : mapDouble(controllerMessage.joystick2.y, -1, 1, -MAX_DIST, MAX_DIST);

        targetXY = {x_pos, y_pos};
        targetXY = createBarrier(targetXY); // creates a barrier such that arm won't hit robot body
        targetPose = inverseKinematics(targetXY);

        updateSetpoints(forward + turn, forward - turn, targetPose.theta1, targetPose.theta2); // theta1 and theta2 being for the arm links
    }
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
    updateSetpoints(0, 0, 0, );
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
    float dtheta = r / (2 * bb) * (dPhiR - dPhiL);
    float dx = r / 2.0 * (cos(robotMessage.theta) * dPhiR + cos(robotMessage.theta) * dPhiL);
    float dy = r / 2.0 * (sin(robotMessage.theta) * dPhiR + sin(robotMessage.theta) * dPhiL);

    // Update robot message
    robotMessage.millis = millis();
    robotMessage.x += dx;
    robotMessage.y += dy;
    robotMessage.theta += dtheta;
}
