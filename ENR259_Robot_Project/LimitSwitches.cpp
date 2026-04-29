#include "Config.h"
#include "LimitSwitches.h"
#include <Arduino.h>

namespace {

struct DebouncedSwitch {
  int pin;
  bool rawPressed;
  bool stablePressed;
  bool clickLatched;
  unsigned long lastRawChange;
};

DebouncedSwitch firstSwitch = { FIRST_LIMIT_PIN, false, false, false, 0 };
DebouncedSwitch backSwitch  = { BACK_LIMIT_PIN,  false, false, false, 0 };

bool readPressedRaw(int pin) {
  // INPUT_PULLUP makes the switch read LOW when pressed.
  return digitalRead(pin) == LOW;
}

void updateOneSwitch(DebouncedSwitch &sw) {
  unsigned long now = millis();
  bool newRawPressed = readPressedRaw(sw.pin);

  if (newRawPressed != sw.rawPressed) {
    sw.rawPressed = newRawPressed;
    sw.lastRawChange = now;
  }

  if (now - sw.lastRawChange >= LIMIT_DEBOUNCE_MS &&
      sw.stablePressed != sw.rawPressed) {
    sw.stablePressed = sw.rawPressed;

    if (sw.stablePressed) {
      sw.clickLatched = true;

      if (DEBUG_MISSION) {
        Serial.print("Limit switch clicked on pin ");
        Serial.println(sw.pin);
      }
    }
  }
}

bool consumeClick(DebouncedSwitch &sw) {
  if (!sw.clickLatched) {
    return false;
  }

  sw.clickLatched = false;
  return true;
}

}  // namespace

void initLimitSwitches() {
  pinMode(FIRST_LIMIT_PIN, INPUT_PULLUP);
  pinMode(BACK_LIMIT_PIN, INPUT_PULLUP);

  firstSwitch.rawPressed = readPressedRaw(FIRST_LIMIT_PIN);
  firstSwitch.stablePressed = firstSwitch.rawPressed;
  firstSwitch.clickLatched = false;
  firstSwitch.lastRawChange = millis();

  backSwitch.rawPressed = readPressedRaw(BACK_LIMIT_PIN);
  backSwitch.stablePressed = backSwitch.rawPressed;
  backSwitch.clickLatched = false;
  backSwitch.lastRawChange = millis();

  if (DEBUG_GENERAL) {
    Serial.println("Limit switches initialised with INPUT_PULLUP.");
    Serial.print("First limit pin: ");
    Serial.println(FIRST_LIMIT_PIN);
    Serial.print("Back limit pin: ");
    Serial.println(BACK_LIMIT_PIN);
  }
}

void updateLimitSwitches() {
  updateOneSwitch(firstSwitch);
  updateOneSwitch(backSwitch);
}

void clearLimitSwitchClicks() {
  firstSwitch.clickLatched = false;
  backSwitch.clickLatched = false;
}

bool firstLimitPressed() {
  return firstSwitch.stablePressed;
}

bool backLimitPressed() {
  return backSwitch.stablePressed;
}

bool firstLimitClicked() {
  return consumeClick(firstSwitch);
}

bool backLimitClicked() {
  return consumeClick(backSwitch);
}