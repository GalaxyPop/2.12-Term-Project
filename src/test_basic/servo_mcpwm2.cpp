#include "driver/mcpwm.h"                     
#define MOSFET1 22          //pin to trigger the MOSFET (Output) pin22
#define MOSFET2 23          //pin to trigger the MOSFET (Output) pin22
void setup() {
  // put your setup code here, to run once:
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOSFET1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOSFET2); 

  mcpwm_config_t pwm_config;
    pwm_config.frequency = 100000;    //frequency = 110Hz
    pwm_config.cmpr_a = 50;       //duty cycle of PWMxA = 60.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
  
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,50);
}

void loop() {
 

}