#include "Config.h"
#include "Sorting.h"
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

uint16_t ballPresentClear = 250;
uint16_t ballGoneClear    = 150;

float ambientR = 0.0f;
float ambientG = 0.0f;
float ambientB = 0.0f;
float ambientC = 0.0f;

struct ColorFeatures {
  int rr;
  int gg;
  int bb;
  int cc;

  float rn;
  float gn;
  float bn;

  float dRed;
  float dWhite;
  float dBlue;
};

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

float sqf(float x) {
  return x * x;
}

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

void computeThresholdsFromAmbient() {
  ballPresentClear = (uint16_t)max(220, (int)(ambientC + 160.0f));
  ballGoneClear    = (uint16_t)max(140, (int)(ambientC + 70.0f));

  if (ballGoneClear >= ballPresentClear - 20) {
    ballGoneClear = ballPresentClear - 20;
  }
}

void calibrateAmbientOnce() {
  uint32_t sumR = 0;
  uint32_t sumG = 0;
  uint32_t sumB = 0;
  uint32_t sumC = 0;
  uint16_t count = 0;

  unsigned long startTime = millis();

  while (millis() - startTime < SORT_AMBIENT_CAL_MS) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    sumR += r;
    sumG += g;
    sumB += b;
    sumC += c;
    count++;

    delay(25);
  }

  if (count == 0) {
    count = 1;
  }

  ambientR = (float)sumR / count;
  ambientG = (float)sumG / count;
  ambientB = (float)sumB / count;
  ambientC = (float)sumC / count;

  computeThresholdsFromAmbient();
}

void readAveragedBallSample(uint16_t &rAvg, uint16_t &gAvg, uint16_t &bAvg, uint16_t &cAvg) {
  uint32_t sumR = 0;
  uint32_t sumG = 0;
  uint32_t sumB = 0;
  uint32_t sumC = 0;

  for (int i = 0; i < SORT_SAMPLE_COUNT; i++) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    sumR += r;
    sumG += g;
    sumB += b;
    sumC += c;
    delay(SORT_SAMPLE_GAP_MS);
  }

  rAvg = sumR / SORT_SAMPLE_COUNT;
  gAvg = sumG / SORT_SAMPLE_COUNT;
  bAvg = sumB / SORT_SAMPLE_COUNT;
  cAvg = sumC / SORT_SAMPLE_COUNT;
}

ColorFeatures computeFeatures(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  ColorFeatures f;

  f.rr = max(0, (int)r - (int)ambientR);
  f.gg = max(0, (int)g - (int)ambientG);
  f.bb = max(0, (int)b - (int)ambientB);
  f.cc = max(0, (int)c - (int)ambientC);

  float sumRGB = (float)(f.rr + f.gg + f.bb);

  if (sumRGB > 0.0f) {
    f.rn = f.rr / sumRGB;
    f.gn = f.gg / sumRGB;
    f.bn = f.bb / sumRGB;
  } else {
    f.rn = 0.0f;
    f.gn = 0.0f;
    f.bn = 0.0f;
  }

  f.dRed =
    sqf(f.rn - 0.784f) +
    sqf(f.gn - 0.096f) +
    sqf(f.bn - 0.120f);

  f.dWhite =
    sqf(f.rn - 0.457f) +
    sqf(f.gn - 0.325f) +
    sqf(f.bn - 0.217f);

  f.dBlue =
    sqf(f.rn - 0.307f) +
    sqf(f.gn - 0.331f) +
    sqf(f.bn - 0.360f);

  return f;
}

BallColor classifyColor(const ColorFeatures &f) {
  float sumRGB = (float)(f.rr + f.gg + f.bb);

  if (f.cc < 80 || sumRGB < 50.0f) {
    return COLOR_NONE;
  }

  if (f.rn >= 0.62f &&
      f.rr >= f.gg * 2.0f &&
      f.rr >= f.bb * 1.8f) {
    return COLOR_RED;
  }

  if (f.rn <= 0.40f &&
      f.gn >= 0.28f &&
      f.bn >= 0.31f &&
      f.rr < f.gg &&
      f.rr < f.bb) {
    return COLOR_BLUE;
  }

  if (f.rn >= 0.38f && f.rn <= 0.60f &&
      f.gn >= 0.24f && f.gn <= 0.40f &&
      f.bn >= 0.14f && f.bn <= 0.30f) {
    return COLOR_WHITE;
  }

  float minD = f.dRed;
  BallColor guess = COLOR_RED;

  if (f.dWhite < minD) {
    minD = f.dWhite;
    guess = COLOR_WHITE;
  }

  if (f.dBlue < minD) {
    minD = f.dBlue;
    guess = COLOR_BLUE;
  }

  if (minD < 0.045f) {
    if (guess == COLOR_BLUE && f.bn >= 0.30f && f.rn <= 0.42f) {
      return COLOR_BLUE;
    }
    return guess;
  }

  if (f.rn > 0.60f) {
    return COLOR_RED;
  }

  if (f.rn < 0.42f && f.bn > 0.30f) {
    return COLOR_BLUE;
  }

  if (f.gn > 0.23f && f.bn > 0.13f) {
    return COLOR_WHITE;
  }

  return COLOR_NONE;
}

void printDebug(uint16_t r, uint16_t g, uint16_t b, uint16_t c, const ColorFeatures &f, BallColor color) {
  if (!DEBUG_SORTING) return;

  Serial.println("===== BALL DETECTED =====");
  Serial.print("Detected color: ");
  Serial.println(colorName(color));

  Serial.print("Raw R: "); Serial.println(r);
  Serial.print("Raw G: "); Serial.println(g);
  Serial.print("Raw B: "); Serial.println(b);
  Serial.print("Raw C: "); Serial.println(c);

  Serial.print("Ambient R: "); Serial.println(ambientR, 1);
  Serial.print("Ambient G: "); Serial.println(ambientG, 1);
  Serial.print("Ambient B: "); Serial.println(ambientB, 1);
  Serial.print("Ambient C: "); Serial.println(ambientC, 1);

  Serial.print("Comp R: "); Serial.println(f.rr);
  Serial.print("Comp G: "); Serial.println(f.gg);
  Serial.print("Comp B: "); Serial.println(f.bb);
  Serial.print("Comp C: "); Serial.println(f.cc);

  Serial.print("R frac: "); Serial.println(f.rn, 3);
  Serial.print("G frac: "); Serial.println(f.gn, 3);
  Serial.print("B frac: "); Serial.println(f.bn, 3);

  Serial.print("dRed: "); Serial.println(f.dRed, 4);
  Serial.print("dWhite: "); Serial.println(f.dWhite, 4);
  Serial.print("dBlue: "); Serial.println(f.dBlue, 4);

  Serial.print("ballPresentClear: "); Serial.println(ballPresentClear);
  Serial.print("ballGoneClear: "); Serial.println(ballGoneClear);
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

  calibrateAmbientOnce();
  sortingEnabled = true;
  lastReadTime = millis();
  setSortingState(SORT_IDLE);

  if (DEBUG_GENERAL || DEBUG_SORTING) {
    Serial.println("Sorting initialised.");
    Serial.print("Ambient R: "); Serial.println(ambientR, 1);
    Serial.print("Ambient G: "); Serial.println(ambientG, 1);
    Serial.print("Ambient B: "); Serial.println(ambientB, 1);
    Serial.print("Ambient C: "); Serial.println(ambientC, 1);
    Serial.print("ballPresentClear: "); Serial.println(ballPresentClear);
    Serial.print("ballGoneClear: "); Serial.println(ballGoneClear);
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

      uint16_t rNow, gNow, bNow, cNow;
      tcs.getRawData(&rNow, &gNow, &bNow, &cNow);

      if (cNow >= ballPresentClear) {
        uint16_t r, g, b, c;
        readAveragedBallSample(r, g, b, c);

        ColorFeatures f = computeFeatures(r, g, b, c);
        BallColor color = classifyColor(f);

        if (color != COLOR_NONE) {
          activeTargetAngle = sortAngleForColor(color);
          printDebug(r, g, b, c, f, color);
          sortServo.write(activeTargetAngle);
          setSortingState(SORT_MOVE_TO_TARGET);
        } else {
          if (DEBUG_SORTING) {
            Serial.println("Ball detected but classification was NONE.");
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

      uint16_t rNow, gNow, bNow, cNow;
      tcs.getRawData(&rNow, &gNow, &bNow, &cNow);

      if (cNow <= ballGoneClear) {
        // Ball cleared normally
        setSortingState(SORT_IDLE);
      } else if (now - sortingStateStart >= 3000UL) {
        // Ball still stuck after 3 seconds — retry gate
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
        // Go back to SORT_WAIT_CLEAR to check if ball cleared
        // Reset the state start time so timeout resets too
        setSortingState(SORT_WAIT_CLEAR);
      }
      break;
  }
}