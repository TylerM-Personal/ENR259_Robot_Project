#include "Logger.h"
#include <Arduino.h>

namespace {
  unsigned long lastLogTime = 0;
  constexpr unsigned long LOG_INTERVAL_MS = 100;
}

void initLogger() {
  Serial1.begin(9600);
  Serial1.println("Move       | Time_ms | Yaw      | Target   | Error    | Correction | Left | Right");
  Serial1.println("-----------|---------|----------|----------|----------|------------|------|------");
}

void logDriveData(LogMoveType moveType, float yaw, float targetHeading, float error, float correction, int leftSpeed, int rightSpeed) {
  unsigned long now = millis();
  if (now - lastLogTime < LOG_INTERVAL_MS) return;
  lastLogTime = now;

  // Move type (10 chars)
  const char* label = moveType == LOG_STRAIGHT ? "STRAIGHT  " : "PIVOT     ";
  Serial1.print(label);
  Serial1.print(" | ");

  // Time (7 chars)
  char buf[32];
  sprintf(buf, "%7lu", now);
  Serial1.print(buf);
  Serial1.print(" | ");

  // Yaw (8 chars)
  sprintf(buf, "%8.3f", yaw);
  Serial1.print(buf);
  Serial1.print(" | ");

  // Target (8 chars)
  sprintf(buf, "%8.3f", targetHeading);
  Serial1.print(buf);
  Serial1.print(" | ");

  // Error (8 chars)
  sprintf(buf, "%8.3f", error);
  Serial1.print(buf);
  Serial1.print(" | ");

  // Correction (10 chars)
  sprintf(buf, "%10.3f", correction);
  Serial1.print(buf);
  Serial1.print(" | ");

  // Left speed (4 chars)
  sprintf(buf, "%4d", leftSpeed);
  Serial1.print(buf);
  Serial1.print(" | ");

  // Right speed (4 chars)
  sprintf(buf, "%4d", rightSpeed);
  Serial1.println(buf);
}
