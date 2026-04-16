#pragma once

enum TurnDirection {
  TURN_LEFT_DIR,
  TURN_RIGHT_DIR
};

void initDrivebase();
void updateDrivebase();
void stopDrivebase();

void startStraightMove(float distanceMM, int baseSpeed, int direction, float targetHeading);
void startPivotToHeading(float targetHeading, TurnDirection direction);

bool drivebaseIsBusy();
float getYawAngle();
