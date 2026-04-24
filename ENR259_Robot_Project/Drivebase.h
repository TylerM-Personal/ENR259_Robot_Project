#pragma once

enum TurnDirection {
  TURN_LEFT_DIR,
  TURN_RIGHT_DIR
};

void initDrivebase();
void updateDrivebase();
void stopDrivebase();
void resetYaw();   // call after all inits to zero heading reference
void resetDriveLog();
void printDriveLog();

// durationMs — drives for fixed time, gyro PID corrects heading throughout
void startStraightMove(unsigned long durationMs, int baseSpeed, int direction, float targetHeading);
void startPivotToHeading(float targetHeading, TurnDirection direction);

bool  drivebaseIsBusy();
float getYawAngle();
