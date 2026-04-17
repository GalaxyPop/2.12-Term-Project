#include <Arduino.h>
#include "robot_pinout.h"
#include "MotorDriver.h"
#include "PID.h"
#include "EncoderVelocity.h"
#include "robot_drive.h"
#include "kinematics.h"

MotorDriver motors[NUM_MOTORS] = { {A_DIR1, A_PWM1, 0}, {A_DIR2, A_PWM2, 1},
                                   {B_DIR1, B_PWM1, 2}, {B_DIR2, B_PWM2, 3} };

EncoderVelocity encoders[NUM_MOTORS] = { {ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_60_RPM, 0.2},
                                         {ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER3_A_PIN, ENCODER3_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER4_A_PIN, ENCODER4_B_PIN, CPR_60_RPM, 0.2} };

PID pids[NUM_MOTORS] = { {Kp, Ki, Kd, 0, pidTau, false}, {Kp, Ki, Kd, 0, pidTau, false},
                         {Kp, Ki, Kd, 0, pidTau, false}, {Kp, Ki, Kd, 0, pidTau, false} };

double alpha = 0.1;
double setpoints[NUM_MOTORS] = {THETA1_OFFSET, 0, 0, THETA2_OFFSET};
double velocities[NUM_MOTORS] = {0, 0, 0, 0};
double initial_position[NUM_MOTORS] = {THETA1_OFFSET, 0, 0, THETA2_OFFSET};
double positions[NUM_MOTORS] = {0, 0, 0, 0};
double controlEfforts[NUM_MOTORS] = {0, 0, 0, 0};
double sign[NUM_MOTORS] = {1, -1, 1, -1}; // depends on orientation of motor positive direction

void setupDrive(){
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
        motors[i].setup();
}

void updateSetpoints(double left, double right, double t1, double t2) {
    setpoints[0] += alpha*(t1 - setpoints[0]);
    setpoints[1] = right;
    setpoints[2] = left;
    setpoints[3] += alpha*(t2 - setpoints[3]);
}

void updatePIDs() {
    updateArms(0); // link 1
    updateArms(3); // link 2
    updateWheels(1); // right wheel
    updateWheels(2); // left wheel
}

void updateArms(int i) {
    positions[i] = initial_position[i] + sign[i] * encoders[i].getPosition(); // in rad
    controlEfforts[i] = pids[i].calculateParallel(positions[i], setpoints[i]);
    motors[i].drive(controlEfforts[i]);
}

void updateWheels(int i) {
    velocities[i] = sign[i] * encoders[i].getVelocity(); // in rad/s
    controlEfforts[i] = pids[i].calculateParallel(velocities[i], setpoints[i]);
    motors[i].drive(controlEfforts[i]);
}
