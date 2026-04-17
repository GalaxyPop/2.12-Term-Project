#ifndef ROBOT_PINOUT_H
#define ROBOT_PINOUT_H

//motor pins
// A1 is link 1, A2 is right wheel motor, B1 is left wheel motor, B2 is link 2
// if you are going to change the pinouts here, be sure to change updateSetpoints()
// and updatePIDs() functions in src/robot/robot_drive.cpp
#define NUM_MOTORS 4
#define A_DIR1 39 // link 1
#define A_PWM1 41 // link 1
#define A_DIR2 40 // right wheel motor
#define A_PWM2 42 // right wheel motor
#define B_DIR1 34 // left wheel motor
#define B_PWM1 7 // left wheel motor
#define B_DIR2 3 // link 2
#define B_PWM2 6 // link 2

//encoder pins
#define ENCODER1_A_PIN 1
#define ENCODER1_B_PIN 2
#define ENCODER2_A_PIN 4 // Encoders 2 and 3 are considered the 2 rear motors
#define ENCODER2_B_PIN 5 //
#define ENCODER3_A_PIN 21 //
#define ENCODER3_B_PIN 38 // Encoders 2 and 3 are considered the 2 rear motors
#define ENCODER4_A_PIN 16
#define ENCODER4_B_PIN 15

//IMU pins
#define BNO08X_CS 12
#define BNO08X_INT 13
#define BNO08X_RESET 14

#endif // ROBOT_PINOUT_H
