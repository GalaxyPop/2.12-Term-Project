#ifndef ROBOT_MOTION_CONTROL_H
#define ROBOT_MOTION_CONTROL_H

constexpr float R_WHEEL = 0.096f;   // wheel radius in meters
constexpr float B_BASE = 0.18288f;  // distance from back wheel to center in meters

void updateOdometry();
void setWheelVelocities(float robotVelocity, float k);
void followTrajectory();
void setupIMU();
void scanIMU();

struct euler_t {
    float yaw;
    float pitch;
    float roll;
};

extern euler_t ypr;

#endif
