#include <Arduino.h>
#include <math.h>
#include "robot_autonomous.h"
#include "robot_drive.h"
#include "robot_motion_control.h"
#include "wireless.h"

extern RobotMessage robotMessage;

typedef ActionStatus (*ActionFn)();

struct AutoStep {
    const char *name;
    ActionFn action;
};

static bool autonomousEnabled = true;
static bool obstacleDetected = false;
static int currentStep = 0;

static const float DRIVE_SPEED = 0.18;       // m/s
static const float REVERSE_SPEED = -0.15;    // m/s
static const float TURN_WHEEL_SPEED = 2.0;   // wheel rad/s
static const float DIST_TOLERANCE = 0.03;    // m
static const float ANGLE_TOLERANCE = 0.05;   // rad

static float startX = 0;
static float startY = 0;
static float startTheta = 0;
static unsigned long actionStartMillis = 0;
static bool actionStarted = false;

static void driveForward(double velocity) {
    setWheelVelocities(velocity, 0);
}

static void stopDrive() {
    updateSetpointsWheels(0, 0);
}

static void beginAction() {
    if (actionStarted) return;

    startX = robotMessage.x;
    startY = robotMessage.y;
    startTheta = robotMessage.theta;
    actionStartMillis = millis();
    actionStarted = true;
}

static ActionStatus finishAction() {
    stopDrive();
    actionStarted = false;
    return ACTION_DONE;
}

static float distanceFromStart() {
    const float dx = robotMessage.x - startX;
    const float dy = robotMessage.y - startY;
    return sqrt(dx * dx + dy * dy);
}

static float angleFromStart() {
    float angle = robotMessage.theta - startTheta;
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

static ActionStatus driveDistance(float meters, float velocity) {
    beginAction();

    if (distanceFromStart() >= fabs(meters) - DIST_TOLERANCE) {
        return finishAction();
    }

    driveForward(velocity);
    return ACTION_RUNNING;
}

static ActionStatus turnAngle(float radians) {
    beginAction();

    const float turned = angleFromStart();
    if (fabs(turned) >= fabs(radians) - ANGLE_TOLERANCE) {
        return finishAction();
    }

    const float direction = radians > 0 ? 1.0 : -1.0;
    updateSetpointsWheels(-direction * TURN_WHEEL_SPEED, direction * TURN_WHEEL_SPEED);
    return ACTION_RUNNING;
}

static ActionStatus waitMillis(unsigned long duration) {
    beginAction();

    if (millis() - actionStartMillis >= duration) {
        return finishAction();
    }

    stopDrive();
    return ACTION_RUNNING;
}

static ActionStatus findAndAlignAprilTag1() {
    // Replace this with camera/Jetson alignment. For now, it is a short non-blocking pause.
    return waitMillis(500);
}

static ActionStatus findAndAlignAprilTag2() {
    return waitMillis(500);
}

static ActionStatus navigateToBaseOfSlopeA() {
    return driveDistance(0.50, DRIVE_SPEED);
}

static ActionStatus navigateToPresetTray() {
    return driveDistance(0.35, DRIVE_SPEED);
}

static ActionStatus moveForwardPresetDistance() {
    return driveDistance(0.20, DRIVE_SPEED);
}

static ActionStatus reversePresetDistance() {
    return driveDistance(0.20, REVERSE_SPEED);
}

static ActionStatus turnRight90() {
    return turnAngle(-PI / 2.0);
}

static ActionStatus turnLeft90() {
    return turnAngle(PI / 2.0);
}

static ActionStatus traverseEmptySlopeA_Up() {
    return driveDistance(0.75, DRIVE_SPEED);
}

static ActionStatus traverseEmptySlopeB_Down() {
    return driveDistance(0.75, DRIVE_SPEED);
}

static ActionStatus reverseEmptySlopeB_Up() {
    return driveDistance(0.75, REVERSE_SPEED);
}

static ActionStatus reverseEmptySlopeA_Down() {
    return driveDistance(0.75, REVERSE_SPEED);
}

static ActionStatus traverseLoadedSlopeA_Up() {
    // This is where IMU balancing can be added using ypr.roll.
    return driveDistance(0.75, 0.12);
}

static ActionStatus reverseLoadedSlopeA_Down() {
    return driveDistance(0.75, -0.12);
}

static ActionStatus reverseLoadedSlopeB_Up() {
    return driveDistance(0.75, -0.12);
}

static ActionStatus traverseLoadedSlopeB_Down() {
    return driveDistance(0.75, 0.12);
}

static ActionStatus scanForFoodTrayPrepTag() {
    return waitMillis(500);
}

static ActionStatus scanForDinnerTableTag() {
    return waitMillis(500);
}

static ActionStatus scanForDishwasherTrayTag() {
    return waitMillis(500);
}

static ActionStatus scanForDishwasherTag() {
    return waitMillis(500);
}

static ActionStatus makeInformedAdjustments() {
    return waitMillis(500);
}

static ActionStatus positionChassisForGrip() {
    return driveDistance(0.08, DRIVE_SPEED);
}

static ActionStatus grabTray() {
    // Replace with gripper/arm command once the mechanism interface exists.
    return waitMillis(700);
}

static ActionStatus depositTray() {
    return waitMillis(700);
}

static ActionStatus pathToBaseOfSlopeA() {
    return driveDistance(0.45, DRIVE_SPEED);
}

static ActionStatus pathToDinnerTable() {
    return driveDistance(0.50, DRIVE_SPEED);
}

static ActionStatus pathToDishwasherTray() {
    return driveDistance(0.50, DRIVE_SPEED);
}

static ActionStatus pathToDishwasher() {
    return driveDistance(0.50, DRIVE_SPEED);
}

static AutoStep sequence[] = {
    {"align ramp tag", findAndAlignAprilTag1},
    {"drive to slope A", navigateToBaseOfSlopeA},
    {"empty slope A up", traverseEmptySlopeA_Up},
    {"turn right", turnRight90},
    {"empty slope B down", traverseEmptySlopeB_Down},
    {"reverse slope B up", reverseEmptySlopeB_Up},
    {"turn left", turnLeft90},
    {"reverse slope A down", reverseEmptySlopeA_Down},
    {"empty slope A up", traverseEmptySlopeA_Up},
    {"turn right", turnRight90},
    {"empty slope B down", traverseEmptySlopeB_Down},
    {"drive to tray", navigateToPresetTray},
    {"forward preset", moveForwardPresetDistance},
    {"turn right", turnRight90},
    {"scan tray tag", scanForFoodTrayPrepTag},
    {"position for grip", positionChassisForGrip},
    {"grab tray", grabTray},
    {"reverse preset", reversePresetDistance},
    {"turn right", turnRight90},
    {"align return tag", findAndAlignAprilTag2},
    {"path to slope A", pathToBaseOfSlopeA},
    {"loaded slope A up", traverseLoadedSlopeA_Up},
    {"loaded slope A down", reverseLoadedSlopeA_Down},
    {"loaded slope A up", traverseLoadedSlopeA_Up},
    {"loaded slope A down", reverseLoadedSlopeA_Down},
    {"loaded slope A up", traverseLoadedSlopeA_Up},
    {"turn right", turnRight90},
    {"loaded slope B down", traverseLoadedSlopeB_Down},
    {"reverse loaded slope B up", reverseLoadedSlopeB_Up},
    {"loaded slope B down", traverseLoadedSlopeB_Down},
    {"reverse loaded slope B up", reverseLoadedSlopeB_Up},
    {"loaded slope B down", traverseLoadedSlopeB_Down},
    {"path to dinner table", pathToDinnerTable},
    {"scan dinner tag", scanForDinnerTableTag},
    {"adjust at dinner table", makeInformedAdjustments},
    {"deposit tray", depositTray},
    {"path to dishwasher tray", pathToDishwasherTray},
    {"scan dishwasher tray", scanForDishwasherTrayTag},
    {"adjust at dishwasher tray", makeInformedAdjustments},
    {"grab tray", grabTray},
    {"path to dishwasher", pathToDishwasher},
    {"scan dishwasher", scanForDishwasherTag},
    {"adjust at dishwasher", makeInformedAdjustments},
    {"deposit tray", depositTray},
};

static const int NUM_STEPS = sizeof(sequence) / sizeof(sequence[0]);

void setupAutonomous() {
    resetAutonomousSequence();
}

void runAutonomousSequence() {
    if (!autonomousEnabled || autonomousSequenceComplete()) {
        stopDrive();
        return;
    }

    if (obstacleDetected) {
        stopDrive();
        return;
    }

    ActionStatus status = sequence[currentStep].action();

    if (status == ACTION_DONE) {
        Serial.print("Finished auto step: ");
        Serial.println(sequence[currentStep].name);
        currentStep++;
    } else if (status == ACTION_FAILED) {
        Serial.print("Failed auto step: ");
        Serial.println(sequence[currentStep].name);
        autonomousEnabled = false;
        stopDrive();
    }
}

void resetAutonomousSequence() {
    currentStep = 0;
    actionStarted = false;
    autonomousEnabled = true;
    stopDrive();
}

void setAutonomousEnabled(bool enabled) {
    autonomousEnabled = enabled;
    if (!enabled) stopDrive();
}

bool autonomousSequenceComplete() {
    return currentStep >= NUM_STEPS;
}

void setObstacleDetected(bool detected) {
    // Set obstacle detection to true or false
    obstacleDetected = detected;
}
