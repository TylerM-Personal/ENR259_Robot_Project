#include <Wire.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"

Servo sortServo;
Servo sortGateServo;

// Pins
const int PIN_SORT_SERVO = 20;
const int PIN_SORT_GATE  = 17;

// Angles
const int SORT_RED_ANGLE   = 50;
const int SORT_WHITE_ANGLE = 90;
const int SORT_BLUE_ANGLE  = 180;
const int SORT_HOME_ANGLE  = 90;

const int SORT_GATE_CLOSED = 130;
const int SORT_GATE_OPEN   = 45;

// Timing
const unsigned long STARTUP_DELAY_MS      = 2000UL;
const unsigned long SORT_GATE_OPEN_MS     = 1400UL;
const unsigned long SORT_SETTLE_MS        = 300UL;
const unsigned long SENSOR_READ_DELAY_MS  = 60UL;
const unsigned long BALL_SAMPLE_GAP_MS    = 12UL;

// Averaging
const int BALL_SAMPLE_COUNT = 5;

// Thresholds
uint16_t BALL_PRESENT_CLEAR = 250;
uint16_t BALL_GONE_CLEAR    = 150;

// TCS34725
Adafruit_TCS34725 tcs =
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

enum BallColor {
  COLOR_NONE,
  COLOR_RED,
  COLOR_WHITE,
  COLOR_BLUE
};

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

unsigned long lastReadTime = 0;
bool ballLatched = false;

// Ambient baseline, calibrated once at startup
float ambientR = 0.0f;
float ambientG = 0.0f;
float ambientB = 0.0f;
float ambientC = 0.0f;

float sqf(float x) {
  return x * x;
}

void computeThresholdsFromAmbient() {
  BALL_PRESENT_CLEAR = (uint16_t)max(220, (int)(ambientC + 160.0f));
  BALL_GONE_CLEAR    = (uint16_t)max(140, (int)(ambientC + 70.0f));

  if (BALL_GONE_CLEAR >= BALL_PRESENT_CLEAR - 20) {
    BALL_GONE_CLEAR = BALL_PRESENT_CLEAR - 20;
  }
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
    case COLOR_RED:
      return SORT_RED_ANGLE;
    case COLOR_WHITE:
      return SORT_WHITE_ANGLE;
    case COLOR_BLUE:
      return SORT_BLUE_ANGLE;
    default:
      return SORT_WHITE_ANGLE;
  }
}

void pulseSortGate() {
  Serial.print("Gate opening at angle: ");
  Serial.println(SORT_GATE_OPEN);

  sortGateServo.write(SORT_GATE_OPEN);
  delay(SORT_GATE_OPEN_MS);

  Serial.print("Gate closing at angle: ");
  Serial.println(SORT_GATE_CLOSED);

  sortGateServo.write(SORT_GATE_CLOSED);
  delay(250);
}

void calibrateAmbientOnce() {
  uint32_t sumR = 0;
  uint32_t sumG = 0;
  uint32_t sumB = 0;
  uint32_t sumC = 0;
  uint16_t count = 0;

  unsigned long startTime = millis();

  while (millis() - startTime < STARTUP_DELAY_MS) {
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

  for (int i = 0; i < BALL_SAMPLE_COUNT; i++) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    sumR += r;
    sumG += g;
    sumB += b;
    sumC += c;

    delay(BALL_SAMPLE_GAP_MS);
  }

  rAvg = sumR / BALL_SAMPLE_COUNT;
  gAvg = sumG / BALL_SAMPLE_COUNT;
  bAvg = sumB / BALL_SAMPLE_COUNT;
  cAvg = sumC / BALL_SAMPLE_COUNT;
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

  // RED
  if (f.rn >= 0.62f &&
      f.rr >= f.gg * 2.0f &&
      f.rr >= f.bb * 1.8f) {
    return COLOR_RED;
  }

  // BLUE
  if (f.rn <= 0.40f &&
      f.gn >= 0.28f &&
      f.bn >= 0.31f &&
      f.rr < f.gg &&
      f.rr < f.bb) {
    return COLOR_BLUE;
  }

  // WHITE
  if (f.rn >= 0.38f && f.rn <= 0.60f &&
      f.gn >= 0.24f && f.gn <= 0.40f &&
      f.bn >= 0.14f && f.bn <= 0.30f) {
    return COLOR_WHITE;
  }

  // Distance fallback
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

void printDebug(uint16_t r, uint16_t g, uint16_t b, uint16_t c, const ColorFeatures &f, BallColor color, int targetAngle) {
  Serial.println("===== BALL DETECTED =====");
  Serial.print("Detected color: ");
  Serial.println(colorName(color));

  Serial.print("Raw R: ");
  Serial.println(r);
  Serial.print("Raw G: ");
  Serial.println(g);
  Serial.print("Raw B: ");
  Serial.println(b);
  Serial.print("Raw C: ");
  Serial.println(c);

  Serial.print("Ambient R: ");
  Serial.println(ambientR, 1);
  Serial.print("Ambient G: ");
  Serial.println(ambientG, 1);
  Serial.print("Ambient B: ");
  Serial.println(ambientB, 1);
  Serial.print("Ambient C: ");
  Serial.println(ambientC, 1);

  Serial.print("Comp R: ");
  Serial.println(f.rr);
  Serial.print("Comp G: ");
  Serial.println(f.gg);
  Serial.print("Comp B: ");
  Serial.println(f.bb);
  Serial.print("Comp C: ");
  Serial.println(f.cc);

  Serial.print("R frac: ");
  Serial.println(f.rn, 3);
  Serial.print("G frac: ");
  Serial.println(f.gn, 3);
  Serial.print("B frac: ");
  Serial.println(f.bn, 3);

  Serial.print("dRed: ");
  Serial.println(f.dRed, 4);
  Serial.print("dWhite: ");
  Serial.println(f.dWhite, 4);
  Serial.print("dBlue: ");
  Serial.println(f.dBlue, 4);

  Serial.print("BALL_PRESENT_CLEAR: ");
  Serial.println(BALL_PRESENT_CLEAR);
  Serial.print("BALL_GONE_CLEAR: ");
  Serial.println(BALL_GONE_CLEAR);

  Serial.print("Sorting servo target angle: ");
  Serial.println(targetAngle);
}

void sortOneBall(BallColor color, uint16_t r, uint16_t g, uint16_t b, uint16_t c, const ColorFeatures &f) {
  int targetAngle = sortAngleForColor(color);

  printDebug(r, g, b, c, f, color, targetAngle);

  sortServo.write(targetAngle);
  delay(SORT_SETTLE_MS);

  pulseSortGate();

  Serial.print("Returning sorting servo to white/home angle: ");
  Serial.println(SORT_WHITE_ANGLE);

  sortServo.write(SORT_WHITE_ANGLE);
  delay(200);

  Serial.println("=========================");
  Serial.println();
}

void readAndSort() {
  if (millis() - lastReadTime < SENSOR_READ_DELAY_MS) {
    return;
  }

  lastReadTime = millis();

  uint16_t rNow, gNow, bNow, cNow;
  tcs.getRawData(&rNow, &gNow, &bNow, &cNow);

  if (!ballLatched && cNow >= BALL_PRESENT_CLEAR) {
    uint16_t r, g, b, c;
    readAveragedBallSample(r, g, b, c);

    ColorFeatures f = computeFeatures(r, g, b, c);
    BallColor color = classifyColor(f);

    if (color != COLOR_NONE) {
      sortOneBall(color, r, g, b, c, f);
    } else {
      Serial.println("===== BALL DETECTED =====");
      Serial.println("Classification was NONE");
      Serial.print("Raw R: "); Serial.println(r);
      Serial.print("Raw G: "); Serial.println(g);
      Serial.print("Raw B: "); Serial.println(b);
      Serial.print("Raw C: "); Serial.println(c);
      Serial.print("Comp R: "); Serial.println(f.rr);
      Serial.print("Comp G: "); Serial.println(f.gg);
      Serial.print("Comp B: "); Serial.println(f.bb);
      Serial.print("Comp C: "); Serial.println(f.cc);
      Serial.print("R frac: "); Serial.println(f.rn, 3);
      Serial.print("G frac: "); Serial.println(f.gn, 3);
      Serial.print("B frac: "); Serial.println(f.bn, 3);
      Serial.println();
    }

    ballLatched = true;
  }

  if (ballLatched && cNow <= BALL_GONE_CLEAR) {
    ballLatched = false;
    Serial.println("Ball cleared sensor");
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  sortServo.attach(PIN_SORT_SERVO);
  sortGateServo.attach(PIN_SORT_GATE);

  sortServo.write(SORT_WHITE_ANGLE);
  sortGateServo.write(SORT_GATE_CLOSED);

  if (!tcs.begin()) {
    Serial.println("TCS34725 not found. Check wiring.");
    while (true) {
    }
  }

  Serial.println("Starting ambient calibration...");
  Serial.println("Keep sensor area empty during this time.");
  calibrateAmbientOnce();

  Serial.println("Ambient calibration complete");
  Serial.print("Ambient R: "); Serial.println(ambientR, 1);
  Serial.print("Ambient G: "); Serial.println(ambientG, 1);
  Serial.print("Ambient B: "); Serial.println(ambientB, 1);
  Serial.print("Ambient C: "); Serial.println(ambientC, 1);
  Serial.print("BALL_PRESENT_CLEAR: "); Serial.println(BALL_PRESENT_CLEAR);
  Serial.print("BALL_GONE_CLEAR: "); Serial.println(BALL_GONE_CLEAR);
  Serial.println();

  Serial.println("Constant sorting mode started");
  Serial.println("Defaulted to white position");
  Serial.println();
}

void loop() {
  readAndSort();
}
