#include "Config.h"
#include "Scooper.h"
#include <Servo.h>

namespace {

Servo scooperLeft;
Servo scooperRight;

bool scooperEnabled = false;

enum ScooperState {
  SCOOPER_IDLE,
  SCOOPER_HOLD_UP,
  SCOOPER_HOLD_DOWN
};

ScooperState  scooperState      = SCOOPER_IDLE;
unsigned long scooperStateStart = 0;

void setScooperState(ScooperState nextState) {
  scooperState      = nextState;
  scooperStateStart = millis();

  if (DEBUG_SCOOPER) {
    Serial.print("Scooper state -> ");
    Serial.println(static_cast<int>(scooperState));
  }
}

}  // namespace

void initScooper() {
  scooperLeft.attach(SCOOPER_SERVO_LEFT_PIN);
  scooperRight.attach(SCOOPER_SERVO_RIGHT_PIN);
  delay(100);

  // Go to 90° first — safe midpoint regardless of where servos parked
  scooperLeft.write(90);
  scooperRight.write(90);
  delay(500);   // wait to reach 90°

  // Then go to down position — ready for lane 1
  scooperLeft.write(SCOOPER_LEFT_DOWN_POS);
  scooperRight.write(SCOOPER_RIGHT_DOWN_POS);
  delay(500);    // wait to reach down

  scooperEnabled = true;

  if (DEBUG_SCOOPER) {
    Serial.println("Scooper initialised at DOWN position.");
  }
}

// Move scooper to down position immediately — no state machine
void setScooperDown() {
  if (!scooperEnabled) return;
  scooperLeft.write(SCOOPER_LEFT_DOWN_POS);
  scooperRight.write(SCOOPER_RIGHT_DOWN_POS);
  setScooperState(SCOOPER_IDLE);
}

// Move scooper to up position immediately — no state machine
// Called before pivots so scooper doesn't dig into turf
void setScooperUp() {
  if (!scooperEnabled) return;
  scooperLeft.write(SCOOPER_LEFT_UP_POS);
  scooperRight.write(SCOOPER_RIGHT_UP_POS);
  setScooperState(SCOOPER_IDLE);
}

// Full scooper cycle — lift then return down
// Called at end of each lane
void startScooperCycle() {
  if (!scooperEnabled || scooperState != SCOOPER_IDLE) return;

  scooperLeft.write(SCOOPER_LEFT_UP_POS);
  scooperRight.write(SCOOPER_RIGHT_UP_POS);
  setScooperState(SCOOPER_HOLD_UP);
}

bool scooperIsBusy() {
  return scooperState != SCOOPER_IDLE;
}

void updateScooper() {
  if (!scooperEnabled) return;

  unsigned long elapsed = millis() - scooperStateStart;

  switch (scooperState) {
    case SCOOPER_IDLE:
      break;

    case SCOOPER_HOLD_UP:
      if (elapsed >= SCOOPER_UP_HOLD_MS) {
        scooperLeft.write(SCOOPER_LEFT_DOWN_POS);
        scooperRight.write(SCOOPER_RIGHT_DOWN_POS);
        setScooperState(SCOOPER_HOLD_DOWN);
      }
      break;

    case SCOOPER_HOLD_DOWN:
      if (elapsed >= SCOOPER_DOWN_HOLD_MS) {
        setScooperState(SCOOPER_IDLE);
      }
      break;
  }
}
