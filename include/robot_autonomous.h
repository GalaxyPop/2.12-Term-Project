#ifndef ROBOT_AUTONOMOUS_H
#define ROBOT_AUTONOMOUS_H

enum ActionStatus {
    ACTION_RUNNING,
    ACTION_DONE,
    ACTION_FAILED
};

void setupAutonomous();
void runAutonomousSequence();
void resetAutonomousSequence();
void setAutonomousEnabled(bool enabled);
bool autonomousSequenceComplete();
void setObstacleDetected(bool detected);

#endif
