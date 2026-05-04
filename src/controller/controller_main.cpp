#include <Bounce2.h>
#include "wireless.h"
#include "util.h"
#include "joystick.h"
#include "dpad.h"
#include "display.h"
#include "controller_pinout.h"

ControllerMessage prevControllerMessage;

Joystick joystick1(JOYSTICK1_X_PIN, JOYSTICK1_Y_PIN);
Joystick joystick2(JOYSTICK2_X_PIN, JOYSTICK2_Y_PIN);

Bounce bounce = Bounce();
int servoState = LOW;

void setup() {
    Serial.begin();

    setupWireless();

    joystick1.setup();
    joystick2.setup(); // added second joystick setup

    Serial.println("Setup complete.");

    bounce.attach( BUTTON_R_PIN ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
    bounce.interval(5); // interval in ms

}

void loop() {

    // SERVO TEST BUTTON
    bounce.update();
    // controllerMessage.buttonR = (bounce.changed() && bounce.read() == LOW) ? 
    //                     !controllerMessage.buttonR : controllerMessage.buttonR;
      // <Bounce>.changed() RETURNS true IF THE STATE CHANGED (FROM HIGH TO LOW OR LOW TO HIGH)
    if ( bounce.changed() ) {
        // THE STATE OF THE INPUT CHANGED
        // GET THE STATE
        int deboucedInput = bounce.read();
        // IF THE CHANGED VALUE IS LOW
        if ( deboucedInput == LOW ) {
            // servoState = (servoState==LOW) ? HIGH : LOW;
            // controllerMessage.buttonR = (servoState==LOW) ? false : true;
            controllerMessage.buttonR = !controllerMessage.buttonR;
            Serial.printf("Servo: ", controllerMessage.buttonR);
        }
    }


    // Read and send controller sensors

    // Serial.println(controllerMessage.joystick1.x);
    // Serial.println(controllerMessage.joystick1.y);
    // Serial.println(controllerMessage.joystick2.x);
    // Serial.println(controllerMessage.joystick2.y);

    EVERY_N_MILLIS(50) {
        controllerMessage.millis = millis();
        controllerMessage.joystick1 = joystick1.read();
        controllerMessage.joystick2 = joystick2.read(); // added second joystick reading

        EVERY_N_MILLIS(200) {
            Serial.println(controllerMessage.joystick1.x);
            Serial.println(controllerMessage.joystick2.y);
            Serial.println(controllerMessage.buttonR);
        }

        if (!(prevControllerMessage == controllerMessage)) {
            sendControllerData();
            prevControllerMessage = controllerMessage;
        }
    }
}
