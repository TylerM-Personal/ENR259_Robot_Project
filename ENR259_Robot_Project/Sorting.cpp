#include "Config.h"
#include "Sorting.h"
#include "Logger.h"
#include <Wire.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"

namespace {

Servo sortServo;
Servo sortGateServo;
Adafruit_TCS34725 tcs(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

bool sortingEnabled = false;
unsigned long lastReadTime = 0;

enum BallColor {
  COLOR_NONE,
  COLOR_RED,
  COLOR_WHITE,
  COLOR_BLUE
};

enum SortingState {
  SORT_IDLE,
  SORT_MOVE_TO_TARGET,
  SORT_GATE_HOLD_OPEN,
  SORT_RETURN_HOME,
  SORT_WAIT_CLEAR,
  SORT_RETRY_GATE,
  SORT_RETRY_HOLD_OPEN
};

SortingState sortingState = SORT_IDLE;
unsigned long sortingStateStart = 0;
int activeTargetAngle = SORT_WHITE_POS;

const char* colorName(BallColor color) {
  switch (color) {
    case COLOR_RED:   return "RED";
    case COLOR_WHITE: return "WHITE";
    case COLOR_BLUE:  return "BLUE";
    default:          return "NONE";
  }
}

int sortAngleForColor(BallColor color) {
  switch (color) {
    case COLOR_RED:   return SORT_RED_POS;
    case COLOR_WHITE: return SORT_WHITE_POS;
    case COLOR_BLUE:  return SORT_BLUE_POS;
    default:          return SORT_WHITE_POS;
  }
}

void setSortingState(SortingState nextState) {
  sortingState = nextState;
  sortingStateStart = millis();

  if (DEBUG_SORTING) {
    Serial.print("Sorting state -> ");
    Serial.println(static_cast<int>(sortingState));
  }
}

// Simple raw-value classifier — uses scale factors to equalize channel
// sensitivities of TCS34725, then compares with a noise threshold.
BallColor classifyColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  float gScaled = g * 1.2f;
  float bScaled = b * 1.83f;

  // White — high overall brightness
  if (c > 17000) {
    return COLOR_WHITE;
  }

  // Red — red dominates with 200 margin
  if (r > gScaled + 200 && r > bScaled + 200) {
    return COLOR_RED;
  }

  // Blue — scaled blue dominates with 200 margin
  if (bScaled > gScaled + 200 && bScaled > r + 200) {
    return COLOR_BLUE;
  }

  return COLOR_NONE;
}

// Detect ball presence by overall clear value above a fixed threshold
bool ballPresent(uint16_t c) {
  return c > 1000;
}

bool ballGone(uint16_t c) {
  return c < 600;
}

void printDebug(uint16_t r, uint16_t g, uint16_t b, uint16_t c, BallColor color) {
  if (!DEBUG_SORTING) return;

  Serial.println("===== BALL DETECTED =====");
  Serial.print("Detected color: ");
  Serial.println(colorName(color));
  Serial.print("Raw R: "); Serial.println(r);
  Serial.print("Raw G: "); Serial.println(g);
  Serial.print("Raw B: "); Serial.println(b);
  Serial.print("Raw C: "); Serial.println(c);
  Serial.print("G*1.2: "); Serial.println(g * 1.2f, 1);
  Serial.print("B*1.83: "); Serial.println(b * 1.83f, 1);
  Serial.println();
}

}  // namespace

void initSorting() {
  if (!ENABLE_SORTING) {
    if (DEBUG_GENERAL) {
      Serial.println("Sorting disabled by config.");
    }
    return;
  }

  sortServo.attach(SORTING_SERVO_PIN);
  sortGateServo.attach(SORT_GATE_SERVO_PIN);

  sortServo.write(SORT_WHITE_POS);
  sortGateServo.write(SORT_GATE_CLOSED_POS);
  delay(100);

  Wire.begin();
  Wire.setClock(400000);

  if (!tcs.begin()) {
    sortingEnabled = false;
    if (DEBUG_GENERAL) {
      Serial.println("TCS34725 not found. Sorting disabled.");
    }
    return;
  }

  sortingEnabled = true;
  lastReadTime = millis();
  setSortingState(SORT_IDLE);

  if (DEBUG_GENERAL || DEBUG_SORTING) {
    Serial.println("Sorting initialised (simple classifier).");
  }
}

bool sortingIsEnabled() {
  return sortingEnabled;
}

bool sortingIsBusy() {
  if (!sortingEnabled) return false;
  return sortingState == SORT_MOVE_TO_TARGET ||
         sortingState == SORT_GATE_HOLD_OPEN ||
         sortingState == SORT_RETURN_HOME ||
         sortingState == SORT_RETRY_GATE ||
         sortingState == SORT_RETRY_HOLD_OPEN;
}

void updateSorting() {
  if (!sortingEnabled) {
    return;
  }

  unsigned long now = millis();

  switch (sortingState) {
    case SORT_IDLE: {
      if (now - lastReadTime < SORT_SENSOR_READ_MS) {
        return;
      }
      lastReadTime = now;

      uint16_t r, g, b, c;
      tcs.getRawData(&r, &g, &b, &c);

      // Log every reading regardless of whether ball is present
      logColorData(r, g, b, c);

      if (ballPresent(c)) {
        BallColor color = classifyColor(r, g, b, c);

        if (color != COLOR_NONE) {
          activeTargetAngle = sortAngleForColor(color);
          printDebug(r, g, b, c, color);
          sortServo.write(activeTargetAngle);
          setSortingState(SORT_MOVE_TO_TARGET);
        } else {
          if (DEBUG_SORTING) {
            Serial.println("Ball present but classification was NONE.");
          }
          setSortingState(SORT_WAIT_CLEAR);
        }
      }
      break;
    }

    case SORT_MOVE_TO_TARGET:
      if (now - sortingStateStart >= SORTING_SERVO_DELAY) {
        sortGateServo.write(SORT_GATE_OPEN_POS);
        setSortingState(SORT_GATE_HOLD_OPEN);
      }
      break;

    case SORT_GATE_HOLD_OPEN:
      if (now - sortingStateStart >= SORT_GATE_OPEN_TIME) {
        sortGateServo.write(SORT_GATE_CLOSED_POS);
        sortServo.write(SORT_ORIGIN_POS);
        setSortingState(SORT_RETURN_HOME);
      }
      break;

    case SORT_RETURN_HOME:
      if (now - sortingStateStart >= SORT_RESET_DELAY) {
        setSortingState(SORT_WAIT_CLEAR);
      }
      break;

    case SORT_WAIT_CLEAR: {
      if (now - lastReadTime < SORT_SENSOR_READ_MS) {
        return;
      }
      lastReadTime = now;

      uint16_t r, g, b, c;
      tcs.getRawData(&r, &g, &b, &c);

      if (ballGone(c)) {
        setSortingState(SORT_IDLE);
      } else if (now - sortingStateStart >= 3000UL) {
        if (DEBUG_SORTING) {
          Serial.println("Ball stuck — retrying gate.");
        }
        sortGateServo.write(SORT_GATE_OPEN_POS);
        setSortingState(SORT_RETRY_GATE);
      }
      break;
    }

    case SORT_RETRY_GATE:
      if (now - sortingStateStart >= SORT_GATE_OPEN_TIME) {
        sortGateServo.write(SORT_GATE_CLOSED_POS);
        setSortingState(SORT_RETRY_HOLD_OPEN);
      }
      break;

    case SORT_RETRY_HOLD_OPEN:
      if (now - sortingStateStart >= SORT_RESET_DELAY) {
        setSortingState(SORT_WAIT_CLEAR);
      }
      break;
  }
}
