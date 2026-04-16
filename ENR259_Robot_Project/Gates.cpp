#include "Config.h"
#include "Gates.h"
#include <Servo.h>

namespace {

Servo redGateServo;
Servo whiteGateServo;
Servo blueGateServo;

bool gatesEnabled = false;
GateColor activeGate = GATE_RED;

// one gate opens at a time, then closes again

enum GateState {
  GATE_IDLE,
  GATE_HOLD_OPEN,
  GATE_SETTLE
};

GateState gateState = GATE_IDLE;
unsigned long gateStateStart = 0;

void closeAllGates() {
  redGateServo.write(GATE_CLOSED_POS);
  whiteGateServo.write(GATE_CLOSED_POS);
  blueGateServo.write(GATE_CLOSED_POS);
}

void openSelectedGate(GateColor gateColor) {
  closeAllGates();

  switch (gateColor) {
    case GATE_RED:
      redGateServo.write(GATE_OPEN_POS);
      break;
    case GATE_WHITE:
      whiteGateServo.write(GATE_OPEN_POS);
      break;
    case GATE_BLUE:
      blueGateServo.write(GATE_OPEN_POS);
      break;
  }
}

void setGateState(GateState nextState) {
  gateState = nextState;
  gateStateStart = millis();

  if (DEBUG_GATES) {
    Serial.print("Gate state -> ");
    Serial.println(static_cast<int>(gateState));
  }
}

}  // namespace

void initGates() {
  redGateServo.attach(RED_GATE_SERVO_PIN);
  whiteGateServo.attach(WHITE_GATE_SERVO_PIN);
  blueGateServo.attach(BLUE_GATE_SERVO_PIN);

  closeAllGates();
  gatesEnabled = true;
}

void startGateCycle(GateColor gateColor) {
  if (!gatesEnabled || gateState != GATE_IDLE) {
    return;
  }

  activeGate = gateColor;
  openSelectedGate(activeGate);
  setGateState(GATE_HOLD_OPEN);
}

bool gatesAreBusy() {
  return gateState != GATE_IDLE;
}

void updateGates() {
  if (!gatesEnabled) {
    return;
  }

  unsigned long elapsed = millis() - gateStateStart;

  switch (gateState) {
    case GATE_IDLE:
      break;

    case GATE_HOLD_OPEN:
      if (elapsed >= GATE_OPEN_HOLD_MS) {
        closeAllGates();
        setGateState(GATE_SETTLE);
      }
      break;

    case GATE_SETTLE:
      if (elapsed >= GATE_SETTLE_MS) {
        setGateState(GATE_IDLE);
      }
      break;
  }
}
