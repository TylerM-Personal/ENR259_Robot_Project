#include "Config.h"
#include "Scooper.h"
#include <Servo.h>

namespace {

Servo scooperServo;

bool scooperEnabled = false;

enum ScooperState {
  SCOOPER_IDLE,
  SCOOPER_HOLD_UP,
  SCOOPER_HOLD_DOWN
};

ScooperState scooperState = SCOOPER_IDLE;
unsigned long scooperStateStart = 0;

void setScooperState(ScooperState nextState) {
  scooperState = nextState;
  scooperStateStart = millis();

  if (DEBUG_SCOOPER) {
    Serial.print("Scooper state -> ");
    Serial.println(static_cast<int>(scooperState));
  }
}

}  // namespace

void initScooper() {
  scooperServo.attach(SCOOPER_SERVO_PIN);
  scooperServo.write(SCOOPER_DOWN_POS);   // start down
  scooperEnabled = true;
}

void setScooperDown() {
  if (!scooperEnabled) {
    return;
  }

  scooperServo.write(SCOOPER_DOWN_POS);
  setScooperState(SCOOPER_IDLE);
}

void startScooperCycle() {
  if (!scooperEnabled || scooperState != SCOOPER_IDLE) {
    return;
  }

  // end-of-lane action: lift first
  scooperServo.write(SCOOPER_UP_POS);
  setScooperState(SCOOPER_HOLD_UP);
}

bool scooperIsBusy() {
  return scooperState != SCOOPER_IDLE;
}

void updateScooper() {
  if (!scooperEnabled) {
    return;
  }

  unsigned long elapsed = millis() - scooperStateStart;

  switch (scooperState) {
    case SCOOPER_IDLE:
      break;

    case SCOOPER_HOLD_UP:
      if (elapsed >= SCOOPER_UP_HOLD_MS) {
        scooperServo.write(SCOOPER_DOWN_POS);   // then return down
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