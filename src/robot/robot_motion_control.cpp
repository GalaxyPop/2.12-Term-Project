#include <Arduino.h>
#include "util.h"
#include "robot_drive.h"
#include "EncoderVelocity.h"
#include "wireless.h"
#include "robot_motion_control.h"
#include "kinematics.h"
#include "trajectories.h"
#include "robot_autonomous.h"
#include "servo_control.h"

// #define AUTONOMOUS

extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;
bool joystickOverride = false; // once joystick data is received, manual control owns the robot until reset
bool armPositionControl = false; // if true, arm's position will be controlled by joystick inputs
bool prevArmPositionControl = armPositionControl;

// jointspace and taskspace representations of the arm's target position
JointSpace targetPose;
TaskSpace targetXY;

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
    double left = (robotVelocity - k * B_BASE * robotVelocity) / R_WHEEL;
    double right = 2 * robotVelocity / R_WHEEL  - left;
    updateSetpointsWheels(left, right);
}

// If the robot is reading joystick data, it will follow the joystick input.
// Otherwise it will run the autonomous sequence.
void followTrajectory() {
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

        } else { // joystick controls arm velocity instead of position
            double theta1_dot = mapStick(controllerMessage.joystick2.x, MAX_SPEED);
            double theta2_dot = mapStick(controllerMessage.joystick2.y, MAX_SPEED);
            updateSetpointsArmsVelocity(theta1_dot, theta2_dot);
        }
    }

    // only runs if autonomous mode is enabled and joystick has not taken over
    #ifdef AUTONOMOUS
    if (!joystickOverride) {
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
    float dtheta = R_WHEEL / (2 * B_BASE) * (dPhiR - dPhiL);
    float dx = R_WHEEL / 2.0 * (cos(robotMessage.theta) * dPhiR + cos(robotMessage.theta) * dPhiL);
    float dy = R_WHEEL / 2.0 * (sin(robotMessage.theta) * dPhiR + sin(robotMessage.theta) * dPhiL);

    // Update robot message
    robotMessage.millis = millis();
    robotMessage.x += dx;
    robotMessage.y += dy;
    robotMessage.theta += dtheta;
}
