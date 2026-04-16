#include "Config.h"
#include "Sorting.h"
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Servo.h>

namespace {

Adafruit_TCS34725 tcs(
  TCS34725_INTEGRATIONTIME_154MS,
  TCS34725_GAIN_16X
);

Servo sortGateServo;
Servo sortingServo;

bool sortingReady = false;
unsigned long lastSensorPrint = 0;

uint16_t lastR = 0;
uint16_t lastG = 0;
uint16_t lastB = 0;
uint16_t lastC = 0;

DetectedColor activeColor = NONE;

enum SortingState {
  SORT_WAIT_FOR_BALL,
  SORT_WAIT_BEFORE_GATE,
  SORT_GATE_OPEN,
  SORT_RESET_SORTER,
  SORT_WAIT_FOR_CLEAR
};

SortingState sortingState = SORT_WAIT_FOR_BALL;
unsigned long sortingStateStart = 0;

void setSortingState(SortingState nextState) {
  sortingState = nextState;
  sortingStateStart = millis();

  if (DEBUG_SORTING) {
    Serial.print("Sorting state -> ");
    Serial.println(static_cast<int>(sortingState));
  }
}

// Read the TCS34725 exactly like the standalone sorter logic.
void readColorSensor() {
  tcs.getRawData(&lastR, &lastG, &lastB, &lastC);

  if (DEBUG_SORTING && millis() - lastSensorPrint >= SORT_SENSOR_PRINT_MS) {
    lastSensorPrint = millis();
    Serial.print("R: ");
    Serial.println(lastR);
    Serial.print("G adj: ");
    Serial.println(lastG * 1.2f);
    Serial.print("B adj: ");
    Serial.println(lastB * 1.83f);
    Serial.print("C: ");
    Serial.println(lastC);
    Serial.println();
  }
}

void moveSorterForColor(DetectedColor color) {
  activeColor = color;

  if (color == RED) {
    sortingServo.write(SORT_RED_POS);
    if (DEBUG_SORTING) {
      Serial.println("Red detected");
    }
  } else if (color == WHITE) {
    sortingServo.write(SORT_WHITE_POS);
    if (DEBUG_SORTING) {
      Serial.println("White detected");
    }
  } else if (color == BLUE) {
    sortingServo.write(SORT_BLUE_POS);
    if (DEBUG_SORTING) {
      Serial.println("Blue detected");
    }
  }
}

}  // namespace

DetectedColor detectColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c > 17000) {
    return WHITE;
  }

  if (r > g * 1.2f + 200 && r > b * 1.83f + 200) {
    return RED;
  }

  if (b * 1.83f > g * 1.2f + 200 && b * 1.83f > r + 200) {
    return BLUE;
  }

  return NONE;
}

void initSorting() {
  if (!ENABLE_SORTING) {
    return;
  }

  sortGateServo.attach(SORT_GATE_SERVO_PIN);
  sortingServo.attach(SORTING_SERVO_PIN);

  sortGateServo.write(SORT_GATE_CLOSED_POS);
  sortingServo.write(SORT_ORIGIN_POS);

  if (!tcs.begin()) {
    if (DEBUG_SORTING || DEBUG_GENERAL) {
      Serial.println("TCS34725 not found. Check wiring.");
    }
    sortingReady = false;
    return;
  }

  sortingReady = true;
  setSortingState(SORT_WAIT_FOR_BALL);

  if (DEBUG_SORTING || DEBUG_GENERAL) {
    Serial.println("Sorting module ready.");
  }
}

void updateSorting() {
  if (!ENABLE_SORTING || !sortingReady) {
    return;
  }

  readColorSensor();
  DetectedColor currentColor = detectColor(lastR, lastG, lastB, lastC);
  unsigned long elapsed = millis() - sortingStateStart;

  switch (sortingState) {
    case SORT_WAIT_FOR_BALL:
      if (currentColor != NONE) {
        moveSorterForColor(currentColor);
        setSortingState(SORT_WAIT_BEFORE_GATE);
      }
      break;

    case SORT_WAIT_BEFORE_GATE:
      if (elapsed >= SORTING_SERVO_DELAY) {
        sortGateServo.write(SORT_GATE_OPEN_POS);
        setSortingState(SORT_GATE_OPEN);
      }
      break;

    case SORT_GATE_OPEN:
      if (elapsed >= SORT_GATE_OPEN_TIME) {
        sortGateServo.write(SORT_GATE_CLOSED_POS);
        setSortingState(SORT_RESET_SORTER);
      }
      break;

    case SORT_RESET_SORTER:
      if (elapsed >= SORT_RESET_DELAY) {
        sortingServo.write(SORT_ORIGIN_POS);
        setSortingState(SORT_WAIT_FOR_CLEAR);
      }
      break;

    case SORT_WAIT_FOR_CLEAR:
      // Wait until the ball is actually clear before allowing another sort.
      // This keeps the standalone sorter behavior but avoids retriggering the
      // same ball multiple times while it still sits under the sensor.
      if (currentColor == NONE) {
        activeColor = NONE;
        sortingServo.write(SORT_ORIGIN_POS);
        sortGateServo.write(SORT_GATE_CLOSED_POS);
        if (DEBUG_SORTING) {
          Serial.println("Ball cleared");
        }
        setSortingState(SORT_WAIT_FOR_BALL);
      }
      break;
  }
}
