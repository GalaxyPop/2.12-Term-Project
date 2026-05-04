//         *********************************************************************
//         *                       ESP32 / ESP-WROOM-32                        *
//         *      Complementary and symmetrical PWM - 180 phase shifted       *
//         *   using the MCPWM module (Motor Control Pulse-Width Modulation)   *
//         *   Daniel Engel                                      09/10/2023    *
//         *********************************************************************
//
/* 
This is a demonstration of complementary symmetrical PWM with an ESP32. I wrote it 
to optimally use color-tunable LEDs with a voltage and current limited power supply.

A PWM wave is generated where each output can run from 0 to maximum time in opposite
manner (when one grows, the other gets smaller). TIMER 0 is used counting up and 
down so that two centered symmetrical PWM waveforms are generated. The second GPIO
pin is inverted to create the 180 phase shift. This is done in the setup with the
line: "GPIO.func_out_sel_cfg[LED_3000K].inv_sel = 1;"

The calculation of the PWM parameters is done in such a way that there is no overlap
between the high states of the two output pins.

References from which I drew inspiration:

https://forum.arduino.cc/t/esp32-mcpwm/608899
https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/mcpwm.html
https://github.com/ul-gh/esp32_ps_pwm/tree/master

Boards I had installed in arduino IDE (maybe only "espressif/arduino-esp32" is needed):
		=> espressif/arduino-esp32 v2.0.12		(https://github.com/espressif/arduino-esp32)
		=> Arduino ESP32 Boards by Arduino 2.0.12
  Successfully compiled for "ESP32 DEVKIT V1 DOIT" board.
*/

#include "driver/mcpwm.h"  // "arduino-esp32-master" v2.0.14   (https://github.com/espressif/arduino-esp32/)

#define LED_5700K 13        // LED_5700K pin
#define LED_3000K 12        // LED_3000K pin  (this output pin has to be inverted in setup)
int frequency = 12000;  // chosen to have a nice display on the oscilloscope... 
// the actual frequency is half this value (the counter counts up then down)
mcpwm_config_t pwm_config;  // initialize "pwm_config" structure

const int colorpotPin = 34;
const int brightpotPin = 35;
int colorPotValue = 0;
int brightPotValue = 0;
int colorPotValueMap = 0;
int brightPotValueMap = 0;  // percentage of the sum of the high states LED_5700K and LED_3000K
int valPwm_3000K = 0;       // PWM value for the 3000K LED
int valPwm_5700K = 0;       // PWM value for the 5700K LED

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(500);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LED_5700K);  // initializes gpio "LED_5700K" for MCPWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, LED_3000K);  // initializes gpio "LED_3000K" for MCPWM
  GPIO.func_out_sel_cfg[LED_3000K].inv_sel = 1;       // <= this line inverts the corresponding output
  pwm_config.frequency = frequency;
  pwm_config.cmpr_a = 0;            // Duty cycle of PWMxA
  pwm_config.cmpr_b = (100);        // Note: the duty cycle of PWMxB is inverted (from where "100-x")
  pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;       // creates symetrical vaweforms
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;              //
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  // Configure PWM0A & PWM0B with settings
}

void loop() {
  readInputsValues();
  valPwm_5700K = (brightPotValueMap * colorPotValueMap) / 100;
  valPwm_3000K = brightPotValueMap - valPwm_5700K;
  pwm_config.cmpr_a = valPwm_3000K;          // Duty cycle of PWMxA (HIGH level)
  pwm_config.cmpr_b = (100 - valPwm_5700K);  // The duty cycle of PWMxB is inverted (100-x)
  updatePWM();
  // printValues(); // uncomment if you want display on the serial console...
  delay(10);
}

void readInputsValues() {
  colorPotValue = analogRead(colorpotPin);
  colorPotValueMap = map(colorPotValue, 0, 4095, 0, 100);
  brightPotValue = analogRead(brightpotPin);
  brightPotValueMap = map(brightPotValue, 0, 4095, 0, 100);
}

void updatePWM() {
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  // Configure PWM0A & PWM0B with settings
}

void printValues() {
  Serial.print("  color : ");
  Serial.print(colorPotValueMap);
  Serial.print("  bright : ");
  Serial.print(brightPotValueMap);

  Serial.print("  |||  LED_5700K : ");
  Serial.print(valPwm_5700K);
  Serial.print("  LED_3000K : ");
  Serial.println(valPwm_3000K);
  delay(500);
}