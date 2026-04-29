#pragma once

enum LogMoveType {
  LOG_STRAIGHT,
  LOG_PIVOT
};

void initLogger();
void logDriveData(LogMoveType moveType, float yaw, float targetHeading, float error, float correction, int leftSpeed, int rightSpeed);
