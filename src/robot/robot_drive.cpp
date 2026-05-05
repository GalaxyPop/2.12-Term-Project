#include <Arduino.h>
#include "robot_pinout.h"
#include "MotorDriver.h"
#include "PID.h"
#include "EncoderVelocity.h"
#include "robot_drive.h"
#include "kinematics.h"
#include "util.h"

MotorDriver motors[NUM_MOTORS] = { {A_DIR1, A_PWM1, 0}, {A_DIR2, A_PWM2, 1},
                                   {B_DIR1, B_PWM1, 2}, {B_DIR2, B_PWM2, 3} };

EncoderVelocity encoders[NUM_MOTORS] = { {ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_60_RPM, 0.2},
                                         {ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER3_A_PIN, ENCODER3_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER4_A_PIN, ENCODER4_B_PIN, CPR_60_RPM, 0.2} };

PID pids[NUM_MOTORS] = { {Kp_arm, Ki_arm, Kd_arm, 0, pidTau, false}, {Kp_wheels, Ki_wheels, Kd_wheels, 0, pidTau, false},
                         {Kp_wheels, Ki_wheels, Kd_wheels, 0, pidTau, false}, {Kp_arm, Ki_arm, Kd_arm, 0, pidTau, false} };

double alpha = 0.05;
double torqueToDuty1 = 0.05;
double torqueToDuty2 = 0.10;

double positionSetpoints[NUM_MOTORS] = {THETA1_OFFSET, 0, 0, THETA2_OFFSET};
double velocitySetpoints[NUM_MOTORS] = {0, 0, 0, 0};
double initial_position[NUM_MOTORS] = {THETA1_OFFSET, 0, 0, THETA2_OFFSET};
double positions[NUM_MOTORS] = {0, 0, 0, 0};
double velocities[NUM_MOTORS] = {0, 0, 0, 0};

double controlEfforts[NUM_MOTORS] = {0, 0, 0, 0};
double encoder_sign[NUM_MOTORS] = {-1, -1, 1, 1}; // depends on orientation of motor positive direction
double motor_sign[NUM_MOTORS] = {-1, 1, 1, -1}; // depends on wiring of motor driver

void setupDrive(){
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
        motors[i].setup();
}

void updateSetpointsWheels(double left, double right) {
    velocitySetpoints[1] = right; // right wheel
    velocitySetpoints[2] = left; // left wheel
}

void updateSetpointsArmsPosition(double theta1, double theta2) {
    positionSetpoints[0] += alpha*(theta1 - positionSetpoints[0]); // link 1
    positionSetpoints[3] += alpha*(theta2 - positionSetpoints[3]); // link 2
}

void updateSetpointsArmsVelocity(double theta1_dot, double theta2_dot) {
    velocitySetpoints[0] = theta1_dot; // link 1
    velocitySetpoints[3] = theta2_dot; // link 2
}

void resetArmPositionSetpoints() {
    positionSetpoints[0] = positions[0]; // link 1
    positionSetpoints[3] = positions[3]; // link 2
}

void updatePIDs(bool armPositionControl) {
    updateArms(armPositionControl); // position control for arms if true, velocity control if false
    updateWheels(); // right and left wheels updated with velocity control
}

void updateArms(bool armPositionControl) {
    positions[0] = initial_position[0] + encoder_sign[0] * encoders[0].getPosition(); // in rad
    positions[3] = initial_position[3] + encoder_sign[3] * encoders[3].getPosition(); // in rad

    // gravity feedforward
    double G1, G2;
    computeGravity(positions[0], positions[3], G1, G2);

    if (armPositionControl) {
        controlEfforts[0] = pids[0].calculateParallel(positions[0], positionSetpoints[0]) + torqueToDuty1 * G1;
        controlEfforts[3] = pids[3].calculateParallel(positions[3], positionSetpoints[3]) + torqueToDuty2 * G2;
    } else { // velocity control
        velocities[0] = encoder_sign[0] * encoders[0].getVelocity(); // in rad/s
        velocities[3] = encoder_sign[3] * encoders[3].getVelocity(); // in rad/s

        controlEfforts[0] = pids[0].calculateParallel(velocities[0], velocitySetpoints[0]) + torqueToDuty1 * G1;
        controlEfforts[3] = pids[3].calculateParallel(velocities[3], velocitySetpoints[3]) + torqueToDuty2 * G2;
    }

    motors[0].drive(motor_sign[0] * controlEfforts[0]);
    motors[3].drive(motor_sign[3] * controlEfforts[3]);
}

void updateWheels() {
    velocities[1] = encoder_sign[1] * encoders[1].getVelocity(); // in rad/s
    velocities[2] = encoder_sign[2] * encoders[2].getVelocity(); // in rad/s

    controlEfforts[1] = pids[1].calculateParallel(velocities[1], velocitySetpoints[1]);
    controlEfforts[2] = pids[2].calculateParallel(velocities[2], velocitySetpoints[2]);

    motors[1].drive(controlEfforts[1]);
    motors[2].drive(controlEfforts[2]);
}

void computeGravity(double theta1, double theta2, double &tau1, double &tau2) {
    // Gravity torque for absolute link angles.
    tau1 =
        (LINK_MASS_KG * GRAVITY * (LINK_LENGTH_M / 2.0) * cos(theta1)) +
        (LINK_MASS_KG * GRAVITY * LINK_LENGTH_M * cos(theta1)) +
        (END_EFFECTOR_MASS_KG * GRAVITY * LINK_LENGTH_M * cos(theta1));

    tau2 =
        (LINK_MASS_KG * GRAVITY * (LINK_LENGTH_M / 2.0) * cos(theta2)) +
        (END_EFFECTOR_MASS_KG * GRAVITY * LINK_LENGTH_M * cos(theta2));
}
