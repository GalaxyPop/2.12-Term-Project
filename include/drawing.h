#ifndef DRAWING_H
#define DRAWING_H
#include <Arduino.h>
#include <math.h>
#include "PID.h"
#include "EncoderVelocity.h"
#include "util.h"
#include "MotorDriver.h"
#include "kinematics.h"
#include "joystick.h"
#include "trajectories.h"

#define SETUP_TIME 2500 //ms

#define THETA1_MAX (M_PI/2) //max deviation from vertical
#define THETA2_MAX M_PI*0.9 //max deviation from vertical
//limits pose to be within safe limits
JointSpace poseLimit(JointSpace pose);
#endif