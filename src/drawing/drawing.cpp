#include "drawing.h"
#include "SimpleFilters.h"

// #define SERIAL_MONITOR
#define MATLAB_PLOT

// TODO: Set trajectoryType to HORIZONTAL_LINE, VERTICAL_LINE, CIRCLE, SPIRAL, or JOYSTICK
TrajectoryType trajectoryType = JOYSTICK;

// TODO: Put your gains here!
#define Ti1 0
#define Td1 0
#define Kp1 0
#define alpha1 0
MotorDriver motor1(DIR1, PWM1, 0);
EncoderVelocity encoder1(ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_60_RPM, 0.2);
LeadLagFilter leadLag1(alpha1, Td1, Ti1);
double setpoint1 = 0;
double position1 = 0;
double controlEffort1 = 0;

// TODO: Put your gains here!
#define Ti2 0
#define Td2 0
#define Kp2 0
#define alpha2 0
MotorDriver motor2(DIR2, PWM2, 1);
EncoderVelocity encoder2(ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_60_RPM, 0.2);
LeadLagFilter leadLag2(alpha2, Td2, Ti2);
double setpoint2 = 0;
double position2 = 0;
double controlEffort2 = 0;


JointSpace targetPose = {THETA1_OFFSET, 0.0};//initial setpoint
JointSpace pose = {THETA1_OFFSET, 0}; //current angles on the robot

TaskSpace initialXY = {0, L1 + L2}; //initial position of end effector
TaskSpace nominalXY = {0, (L1 + L2)*0.6}; //nominal position of end effector
TaskSpace targetXY = nominalXY; //where we want the end effector to go
TaskSpace actualXY; //where the end effector actually is


void setup() {
    motor1.setup();
    motor2.setup();
    Serial.begin();
    setupJoystick();

}

void loop() {
    // Update setpoint and kinematics at 200hz
    EVERY_N_MILLIS(5) {
        //get new cartesian setpoint based on trajectory type
        targetXY = updateSetpoint(initialXY, nominalXY, targetXY, trajectoryType, millis());
        //update current XY based on current pose
        actualXY = forwardKinematics(pose);
        //Get new joint setpoint based on cartesian setpoint
        JointSpace newTargetPose = inverseKinematics(targetXY);
        //limit pose to be within safe limits
        targetPose = poseLimit(newTargetPose);
    }

    // Update PID at 2kHz
    EVERY_N_MICROS(500) {
        //get the current position of the end effector based on the encoders
        pose.theta1 = encoder1.getPosition() + THETA1_OFFSET;
        pose.theta2 = -encoder2.getPosition();

        //calculate the control effort using PID
        controlEffort1 = Kp1*leadLag1.calculate(targetPose.theta1-pose.theta1);
        controlEffort2 = Kp2*leadLag2.calculate(targetPose.theta2-pose.theta2);

        //drive the motors
        motor1.drive(controlEffort1);
        motor2.drive(controlEffort2);
    }

    // Print values at 50Hz
    EVERY_N_MILLIS(20) {
            #ifdef SERIAL_MONITOR
            // Print values to serial monitor
                Serial.printf("Target X: %.3f, Y: %.3f   "
                          "Actual X: %.3f, Y: %.3f\n",
                          targetXY.x, targetXY.y,
                          actualXY.x, actualXY.y);
            #endif

            #ifdef MATLAB_PLOT
                Serial.printf("%.2f\t%.2f\t%.2f\t%.2f\n",
                        targetXY.x, targetXY.y,
                        actualXY.x, actualXY.y);
            #endif
    }
}

JointSpace poseLimit(JointSpace pose){
    pose.theta1 = constrain(pose.theta1, -THETA1_MAX + THETA1_OFFSET, THETA1_MAX + THETA1_OFFSET);
    pose.theta2 = constrain(pose.theta2, -THETA2_MAX, THETA2_MAX);
    pose.theta2 = constrain(pose.theta2, -THETA2_MAX/2-pose.theta1 + THETA1_OFFSET, THETA2_MAX/2+pose.theta1 + THETA1_OFFSET);
    return pose;
}
