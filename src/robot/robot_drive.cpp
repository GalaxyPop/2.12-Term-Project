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

double setpoints[NUM_MOTORS] = {THETA1_OFFSET, 0, 0, THETA2_OFFSET}; // for position
// double setpoints[NUM_MOTORS] = {0, 0, 0, 0}; // for velocity
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
    setpoints[1] = right; // right wheel
    setpoints[2] = left; // left wheel
}

void updateSetpointsArms(double theta1, double theta2) {
    setpoints[0] += alpha*(theta1 - setpoints[0]); // link 1
    setpoints[3] += alpha*(theta2 - setpoints[3]); // link 2
}

void updatePIDs() {
    updateArms(true); // position control for arms if true, velocity control if false
    updateWheels(1); // right wheel
    updateWheels(2); // left wheel
}

void updateArms(bool positionControl) {
    // if positionControl is true, setpoint is interpreted as desired position
    // otherwise, setpoint is interpreted as desired velocity
    positions[0] = initial_position[0] + encoder_sign[0] * encoders[0].getPosition(); // in rad
    positions[3] = initial_position[3] + encoder_sign[3] * encoders[3].getPosition(); // in rad

    velocities[0] = encoder_sign[0] * encoders[0].getVelocity(); // in rad/s
    velocities[3] = encoder_sign[3] * encoders[3].getVelocity(); // in rad/s

    // gravity feedforward
    double G1, G2;
    computeGravity(positions[0], positions[3], G1, G2);

    if (positionControl) {
        controlEfforts[0] = pids[0].calculateParallel(positions[0], setpoints[0]) + torqueToDuty1 * G1;
        controlEfforts[3] = pids[3].calculateParallel(positions[3], setpoints[3]) + torqueToDuty2 * G2;
    } else { // velocity control
        controlEfforts[0] = pids[0].calculateParallel(velocities[0], setpoints[0]) + torqueToDuty1 * G1;
        controlEfforts[3] = pids[3].calculateParallel(velocities[3], setpoints[3]) + torqueToDuty2 * G2;
    }

    motors[0].drive(motor_sign[0] * controlEfforts[0]);
    motors[3].drive(motor_sign[3] * controlEfforts[3]);
}

void updateWheels(int i) {
    velocities[i] = encoder_sign[i] * encoders[i].getVelocity(); // in rad/s
    controlEfforts[i] = pids[i].calculateParallel(velocities[i], setpoints[i]);
    motors[i].drive(controlEfforts[i]);
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
