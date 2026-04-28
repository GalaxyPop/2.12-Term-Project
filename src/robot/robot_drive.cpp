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

PID pids[NUM_MOTORS] = { {Kp_arm, Ki_arm, Kd_arm, 0, pidTau, false}, {Kp_wheels, Ki_wheels, Kd_wheels, 0, pidTau, false},
                         {Kp_wheels, Ki_wheels, Kd_wheels, 0, pidTau, false}, {Kp_arm, Ki_arm, Kd_arm, 0, pidTau, false} };

double alpha = 0.05;
double setpoints[NUM_MOTORS] = {THETA1_OFFSET, 0, 0, THETA2_OFFSET};
double velocities[NUM_MOTORS] = {0, 0, 0, 0};
double initial_position[NUM_MOTORS] = {THETA1_OFFSET, 0, 0, THETA2_OFFSET};
double positions[NUM_MOTORS] = {0, 0, 0, 0};
double controlEfforts[NUM_MOTORS] = {0, 0, 0, 0};
double encoder_sign[NUM_MOTORS] = {-1, -1, 1, 1}; // depends on orientation of motor positive direction
double motor_sign[NUM_MOTORS] = {-1, 1, 1, -1}; // depends on wiring of motor driver

void setupDrive(){
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
        motors[i].setup();
}

void updateSetpointsWheels(double left, double right) {
    setpoints[1] = right; // right wheel
    setpoints[2] = left; // left wheel
}

void updateSetpointsArms(double theta1, double theta2) {
    setpoints[0] += alpha*(theta1 - setpoints[0]); // link 1
    setpoints[3] += alpha*(theta2 - setpoints[3]); // link 2
}

void updatePIDs() {
    updateArms(0); // link 1
    updateArms(3); // link 2
    updateWheels(1); // right wheel
    updateWheels(2); // left wheel
}

void updateArms(int i) {
    positions[i] = initial_position[i] + encoder_sign[i] * encoders[i].getPosition(); // in rad
    controlEfforts[i] = pids[i].calculateParallel(positions[i], setpoints[i]);
    motors[i].drive(motor_sign[i]*controlEfforts[i]);
}

void updateWheels(int i) {
    velocities[i] = encoder_sign[i] * encoders[i].getVelocity(); // in rad/s
    controlEfforts[i] = pids[i].calculateParallel(velocities[i], setpoints[i]);
    motors[i].drive(controlEfforts[i]);
}
