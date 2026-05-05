#ifndef ROBOT_IMU_H
#define ROBOT_IMU_H

struct euler_t {
    float yaw;
    float pitch;
    float roll;
};

extern euler_t ypr; // global variable to hold the latest IMU readings in Euler angles (yaw, pitch, roll)

void setupIMU();
void scanIMU();

#endif // ROBOT_IMU_H
