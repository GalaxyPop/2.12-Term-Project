#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#define NUM_MOTORS 4

#define Kp_wheels 0.25
#define Ki_wheels 0.01
#define Kd_wheels 0
#define Kp_arm 1
#define Ki_arm 0
#define Kd_arm 0.05
#define pidTau 0.1

#define MAX_FORWARD 6
#define MAX_TURN 3

void setupDrive();
void updateSetpointsWheels(double left, double right);
void updateSetpointsArmsPosition(double theta1, double theta2);
void updateSetpointsArmsVelocity(double theta1_dot, double theta2_dot);
void resetArmPositionSetpoints();
void updatePIDs(bool armPositionControl);
void updateArms(bool armPositionControl);
void updateWheels();
void computeGravity(double theta1, double theta2, double &tau1, double &tau2);

#endif // ROBOT_DRIVE_H
