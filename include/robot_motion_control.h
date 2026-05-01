#ifndef ROBOT_MOTION_CONTROL_H
#define ROBOT_MOTION_CONTROL_H

// wheel radius in meters
#define r 0.096
// distance from back wheel to center in meters
#define bb 0.18288

void followTrajectory();
void updateOdometry();
void setupIMU();
void scanIMU();

struct euler_t {
    float yaw;
    float pitch;
    float roll;
};

extern euler_t ypr;

#endif
