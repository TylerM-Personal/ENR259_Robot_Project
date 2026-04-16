#include "Config.h"
#include "Conveyor.h"

void initConveyor() {
  pinMode(CONVEYOR_PIN, OUTPUT);
  analogWriteFrequency(CONVEYOR_PIN, 20000);  // 20 kHz, inaudible & MOSFET-friendly
  analogWrite(CONVEYOR_PIN, 0);
}

void setConveyorSpeed(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(CONVEYOR_PIN, speed);
}

void conveyorOn() {
  setConveyorSpeed(CONVEYOR_SPEED);
}

void conveyorOff() {
  setConveyorSpeed(0);
}