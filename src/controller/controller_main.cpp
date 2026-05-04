#include <Bounce2.h>
#include "wireless.h"
#include "util.h"
#include "joystick.h"
#include "dpad.h"
#include "display.h"
#include "controller_pinout.h"

ControllerMessage prevControllerMessage;
bool prevButtonRPressed = false;

Joystick joystick1(JOYSTICK1_X_PIN, JOYSTICK1_Y_PIN);
Joystick joystick2(JOYSTICK2_X_PIN, JOYSTICK2_Y_PIN);

void setup() {
    Serial.begin();

    setupWireless();

    joystick1.setup();
    joystick2.setup(); // added second joystick setup

    Serial.println("Setup complete.");
}

void loop() {
    // Read and send controller sensors

    EVERY_N_MILLIS(50) {
        controllerMessage.millis = millis();
        controllerMessage.joystick1 = joystick1.read();
        controllerMessage.joystick2 = joystick2.read(); // added second joystick reading

        bool buttonRPressed = analogRead(BUTTON_R_PIN) == 0;
        if (buttonRPressed && !prevButtonRPressed) {
            controllerMessage.buttonR = !controllerMessage.buttonR;
        }
        prevButtonRPressed = buttonRPressed;

        EVERY_N_MILLIS(200) {
            // Serial.println(controllerMessage.joystick1.x);
            // Serial.println(controllerMessage.joystick2.y);
            Serial.println(controllerMessage.buttonR);
        }

        if (!(prevControllerMessage == controllerMessage)) {
            sendControllerData();
            prevControllerMessage = controllerMessage;
        }
    }
}
