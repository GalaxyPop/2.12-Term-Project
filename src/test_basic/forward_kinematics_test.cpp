#include "EncoderVelocity.h"
#include "pinout.h"
#include "EveryNMillis.h"
#include "EveryNMicros.h"
#include "kinematics.h"

#define PRINT_DELAY 30 // Delay between printing to serial in milliseconds
#define READ_DELAY 1000 // Delay between reading encoders in microseconds

JointSpace state;
TaskSpace point;

// Encoder velocity readers
EncoderVelocity encoder1(ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_60_RPM);
EncoderVelocity encoder2(ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_60_RPM);

void setup() {
    Serial.begin();
}

void loop(){
    // Read encoders every READ_DELAY microseconds
    EVERY_N_MICROS(READ_DELAY) {
        state.theta1 = encoder1.getPosition() + THETA1_OFFSET;
        state.theta2 = -encoder2.getPosition();
    }

    // Print every PRINT_DELAY milliseconds
    EVERY_N_MILLIS(PRINT_DELAY) {
        point = forwardKinematics(state);
        Serial.printf("%.2f\t%.2f\n", point.x, point.y);
    }
}