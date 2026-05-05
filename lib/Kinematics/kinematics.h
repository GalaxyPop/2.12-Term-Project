#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>

#define L_HOLE 2.4   // cm
#define L1 (L_HOLE * 10.0)
#define L2 (L_HOLE * 10.0)

const double GRAVITY = 9.81;
const double LINK_LENGTH_M = L_HOLE * 10.0 / 100.0; // in m
const double LINK_MASS_KG = 0.175; // mass of each link in kg
const double END_EFFECTOR_MASS_KG = 0.482;

#define DELTA_MAX 0
#define MAX_DIST (L1 + L2 - DELTA_MAX)
#define MIN_DIST (L1 + L2) * 0.25
#define MAX_SPEED 1 // in rad/s

#define THETA1_OFFSET M_PI / 2.0
#define THETA2_OFFSET M_PI / 2.0

struct JointSpace {
    double theta1; // radians
    double theta2; // radians
};

struct TaskSpace {
    double x; // cm
    double y; // cm
};

TaskSpace forwardKinematics(JointSpace state);
JointSpace inverseKinematics(TaskSpace point);

#endif
