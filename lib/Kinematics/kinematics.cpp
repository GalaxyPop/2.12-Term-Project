#include "kinematics.h"
#include <math.h>
#include <Arduino.h>

TaskSpace forwardKinematics(JointSpace state) {
    // Initializes a TaskSpace variable called point
    TaskSpace point;

    // TODO 1: Modify the two lines below to use the forward kinematics equations you derived.
    // You may need the variables: L1, L2, state.theta1, state.theta2
    // as well as the functions: double cos(double x), double sin(double x), double tan(double x).
    // These variables and functions are already defined, you don't need to define them here.
    point.x = L1*cos(state.theta1) + L2*cos(state.theta2);
    point.y = L1*sin(state.theta1) + L2*sin(state.theta2);

    return point;
}

JointSpace inverseKinematics(TaskSpace point) {
    // Initializes a JointSpace variable called state
    JointSpace state;

    double x2 = point.x * point.x;
    double y2 = point.y * point.y;
    double r2 = x2 + y2;

    // from Law of Cosines for theta2
    double c2 = (r2 - L1*L1 - L2*L2) / (2 * L1 * L2);
    c2 = constrain(c2, -1.0, 1.0); // to prevent numerical issues with acos when point is at the edge of the workspace

    // --- choose elbow configuration ---
    bool elbowUp = true;   // change this to false for elbow-down

    // elbowUp requires negative s2
    double s2 = -sqrt(fmax(0.0, 1 - c2*c2)); // to prevent numerical issues with sqrt when point is at the edge of the workspace
    if (!elbowUp) s2 = -s2; // turn to elbow-down solution

    double theta2_rel = atan2(s2, c2); // [-pi, pi]

    // --- compute theta1 and theta2, with both being absolute link angles ---
    state.theta1 = atan2(point.y, point.x)
                 - atan2(L2 * s2, L1 + L2 * c2);
    state.theta2 = state.theta1 + theta2_rel;

    // --- normalize angle theta1 between [3pi/2, -pi/2] ---
    state.theta1 = atan2(sin(state.theta1), cos(state.theta1));
    if (state.theta1 < -M_PI/2) {
        // if theta1 is less than -pi/2, add 2*pi radians
        // theta1 needs to be in the range [3pi/2, -pi/2] to not hit the body
        state.theta1 += 2.0*M_PI;
    }

    return state;
}
