#include "Config.h"
#include "Drivebase.h"
#include "Gates.h"
#include "Mission.h"
#include "Scooper.h"
#include "Sorting.h"
#include "Conveyor.h"
#include "Logger.h"

void setup() {
  delay(5000); // temp to give time to connect to bluetooth
  Serial.begin(115200);
  initLogger();
  // while (!Serial && millis() < 3000) {}

  initDrivebase();
  initSorting();
  initScooper();
  initGates();
  initConveyor();

  delay(500);
  resetYaw();
  initMission();

  if (DEBUG_GENERAL) {
    Serial.println("ENR259_Robot_Project started.");
    Serial.println("Time based driving.");
    Serial.println("Encoders disabled.");
    Serial.println("Scooper servos on pins 4 and 5.");
    Serial.println(ENABLE_SORTING ? "Sorting enabled." : "Sorting disabled.");
    Serial.println("Yaw zeroed — ready to run.");
  }
}

void loop() {
  updateDrivebase();
  updateSorting();
  updateScooper();
  updateGates();
  updateMission();
}
