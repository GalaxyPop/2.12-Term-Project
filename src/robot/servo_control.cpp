#include "servo_control.h"
#include <Arduino.h>
#include "robot_pinout.h"
#include "mcpwm.h"

void writeServoUS(int pulse_us) {
    pulse_us = constrain(pulse_us, 1000, 2000);

    mcpwm_set_duty_in_us(
        MCPWM_UNIT_0,
        MCPWM_TIMER_0,
        MCPWM_OPR_A,
        pulse_us
    );
}

void setupServo() {
    // Route MCPWM0A to GPIO 43
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PIN);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;              // 50 Hz servo PWM
    pwm_config.cmpr_a = 0;                  // duty A starts at 0
    pwm_config.cmpr_b = 0;                  // unused
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void gripClose() {
    writeServoUS(GRIP_POS_US);
}

void gripOpen() {
    writeServoUS(OPEN_POS_US);
}
