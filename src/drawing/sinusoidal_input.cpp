#include <Arduino.h>
#include "PID.h"
#include "SimpleFilters.h"
#include "EncoderVelocity.h"
#include "pinout.h"
#include "util.h"
#include "MotorDriver.h"

// #define SERIAL_MONITOR
#define MATLAB_PLOT

#define MOTOR1
//#define MOTOR2

#ifdef MOTOR1
    // TODO: Put your gains here!
    #define Ti1 0
    #define Td1 0
    #define Kp1 0
    #define alpha1 0
    MotorDriver motor1(DIR1, PWM1, 0);  
    EncoderVelocity encoder1(ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_60_RPM, 0.2);
    LeadLagFilter leadLag1(alpha1, Td1, Ti1);
    double setpoint1 = 0;
    double position1 = 0;
    double controlEffort1 = 0;
    
    #define MAX_FREQ_1 10.0 //rad/s
    #define MAX_AMPLITUDE_1 M_PI/8 //rad
    double freq1 = MAX_FREQ_1/2;
    double amplitude1 = MAX_AMPLITUDE_1/4;
#endif

#ifdef MOTOR2
    // TODO: Put your gains here!
    #define Ti2 0
    #define Td2 0
    #define Kp2 0
    #define alpha2 0
    MotorDriver motor2(DIR2, PWM2, 1);  
    EncoderVelocity encoder2(ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_60_RPM, 0.2);
    LeadLagFilter leadLag2(alpha2, Td2, Ti2);
    double setpoint2 = 0;
    double position2 = 0;
    double controlEffort2 = 0;

    #define MAX_FREQ_2 15 //rad/s
    #define MAX_AMPLITUDE_2 M_PI/4 //rad
    double freq2 = MAX_FREQ_2/2;
    double amplitude2 = MAX_AMPLITUDE_2/6;

#endif

void setup() {
    #ifdef MOTOR1
        motor1.setup();
    #endif
    #ifdef MOTOR2
        motor2.setup();
    #endif
    Serial.begin(); 
}

void loop() {
    // Update sinusiodal setpoint at 2kHz
    EVERY_N_MICROS(500) {
        #ifdef MOTOR1
            setpoint1 = amplitude1*sin(freq1*millis()/1000.0);
        #endif
        #ifdef MOTOR2
            setpoint2 = amplitude2*sin(freq2*millis()/1000.0);
        #endif
    }

    //update PID at 5khz
    EVERY_N_MICROS(200) {
        #ifdef MOTOR1
            position1 = encoder1.getPosition();
            controlEffort1 = Kp1*leadLag1.calculate(setpoint1-position1);
            motor1.drive(controlEffort1);
        #endif
        #ifdef MOTOR2
            position2 = -encoder2.getPosition();
            controlEffort2 = Kp2*leadLag2.calculate(setpoint2-position2);
            motor2.drive(controlEffort2);
        #endif
    }


    // Print values at 50Hz
    EVERY_N_MILLIS(20) {
            // Print values to serial monitor
            #ifdef SERIAL_MONITOR
            // Print values to serial monitor
            #ifdef MOTOR1
                Serial.printf("Setpoint (rad): %.2f, Position (rad): %.2f, Control Effort: %.2f\n",
                          setpoint1, position1, controlEffort1);
            #endif
            #ifdef MOTOR2
                Serial.printf("Setpoint (rad): %.2f, Position (rad): %.2f, Control Effort: %.2f\n",
                          setpoint2, position2, controlEffort2);
            #endif
            #endif

            #ifdef MATLAB_PLOT
                #ifdef MOTOR1
                    Serial.printf("%.3f\t%.3f\t%.3f\n", millis()/1000.0, setpoint1, position1);
                #endif
                #ifdef MOTOR2
                    Serial.printf("%.3f\t%.3f\t%.3f\n", millis()/1000.0, setpoint2, position2);
                #endif  
            #endif
        }
}
