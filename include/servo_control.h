#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

void gripClose();
void gripOpen();
void setupServo();
void writeServoUS(int pulse_us);

#endif
