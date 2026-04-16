#include "Config.h"
#include "Drivebase.h"
#include "Gates.h"
#include "Mission.h"
#include "Scooper.h"
#include "Sorting.h"
#include "Conveyor.h"


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }

  initDrivebase();
  initSorting();
  initScooper();
  initGates();
  initMission();

  if (DEBUG_GENERAL) {
    Serial.println("ENR259_Robot_Project started.");
    Serial.println("Drivebase uses gyro + encoders from message (4).");
    Serial.println("Sorting module runs continuously during the mission.");
    Serial.println("Final top-wall dropoff uses 3 direct gates: red, white, blue.");
  }

  initConveyor(); //initializing conveyor motor
  conveyorOn();
}

void loop() {
  updateDrivebase();
  updateSorting();
  updateScooper();
  updateGates();
  updateMission();
}
