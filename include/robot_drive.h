#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#define NUM_MOTORS 4

#define Kp_wheels 0.25
#define Ki_wheels 0.01
#define Kd_wheels 0
#define Kp_arm 1
#define Ki_arm 0.02
#define Kd_arm 0.05
#define pidTau 0.1

#define MAX_FORWARD 6
#define MAX_TURN 3

void setupDrive();
void updateSetpointsWheels(double left, double right);
void updateSetpointsArms(double theta1, double theta2);
void resetArmSetpoints(double t1, double t2);
void updatePIDs();
void updateArms(bool positionControl);
void updateWheels(int i);
void computeGravity(double theta1, double theta2, double &tau1, double &tau2);

// Map body-frame (v, w) [m/s, rad/s] to left/right wheel setpoints [rad/s].
void v_omega_to_wheels(double v, double w);

#endif // ROBOT_DRIVE_H
