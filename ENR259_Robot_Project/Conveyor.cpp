#include "Config.h"
#include "Conveyor.h"

void initConveyor() {
  pinMode(CONVEYOR_PIN, OUTPUT);
  analogWriteFrequency(CONVEYOR_PIN, 20000);
  analogWrite(CONVEYOR_PIN, 0);
}

void setConveyorSpeed(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(CONVEYOR_PIN, speed);
}

void conveyorOn() {
  // Run at the configured steady speed.
  // The previous version hard-coded 128, which ignored CONVEYOR_SPEED.
  setConveyorSpeed(CONVEYOR_SPEED);
}

void conveyorOff() {
  setConveyorSpeed(0);
}
