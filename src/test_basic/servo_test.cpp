#include <Arduino.h>
#include "controller_pinout.h"

/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-pwm-arduino-ide/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.  
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

// the number of the LED pin
const int servoPin = SERVO;  // A0 --> corresponds to GPIO 1

void setup() {
  // set the LED as an output
  pinMode(servoPin, OUTPUT);
  Serial.begin();

}

void loop(){
  // increase the LED brightness
  for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){   
    // changing the LED brightness with PWM
    analogWrite(servoPin, dutyCycle);
    delay(100);
      Serial.println(dutyCycle);
  }

  // decrease the LED brightness
  for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
    // changing the LED brightness with PWM
    analogWrite(servoPin, dutyCycle);
    delay(100);
  }
}