#include <Wire.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"

Servo sortServo;
Servo sortGateServo;
Servo redStoreServo;
Servo whiteStoreServo;
Servo blueStoreServo;

// Uno pins
const int PIN_SORT_SERVO  = 5;
const int PIN_SORT_GATE   = 4;
const int PIN_RED_STORE   = 11;
const int PIN_WHITE_STORE = 12;
const int PIN_BLUE_STORE  = 13;

// Sorting angles
const int SORT_RED_ANGLE   = 70;
const int SORT_WHITE_ANGLE = 125;
const int SORT_BLUE_ANGLE  = 175;
const int SORT_HOME_ANGLE  = 125;

// Sorting gate angles
const int SORT_GATE_CLOSED = 5;
const int SORT_GATE_OPEN   = 45;

// Storage gate angles
const int RED_STORE_CLOSED   = 10;
const int RED_STORE_OPEN     = 95;

const int WHITE_STORE_CLOSED = 10;
const int WHITE_STORE_OPEN   = 95;

const int BLUE_STORE_CLOSED  = 10;
const int BLUE_STORE_OPEN    = 95;

// Timing
const unsigned long SORT_TIME_MS         = 30000UL;
const unsigned long STORAGE_INTERVAL_MS  = 5000UL;
const unsigned long SORT_GATE_OPEN_MS    = 700UL;
const unsigned long SORT_SETTLE_MS       = 300UL;
const unsigned long STORE_GATE_OPEN_MS   = 1500UL;
const unsigned long SENSOR_READ_DELAY_MS = 60UL;

// TCS34725
Adafruit_TCS34725 tcs =
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Tune these if needed
const uint16_t BALL_PRESENT_CLEAR = 350;
const uint16_t BALL_GONE_CLEAR    = 220;

enum BallColor {
  COLOR_NONE,
  COLOR_RED,
  COLOR_WHITE,
  COLOR_BLUE
};

unsigned long sortStartTime = 0;
unsigned long lastReadTime = 0;
bool sortPhaseDone = false;
bool dumpPhaseDone = false;
bool ballLatched = false;

BallColor classifyColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < BALL_PRESENT_CLEAR) {
    return COLOR_NONE;
  }

  uint16_t maxRGB = max(r, max(g, b));
  uint16_t minRGB = min(r, min(g, b));

  if (minRGB == 0) {
    minRGB = 1;
  }

  float spread = (float) maxRGB / (float) minRGB;

  if (spread < 1.22f) {
    return COLOR_WHITE;
  }

  if (r > g * 1.22f && r > b * 1.30f) {
    return COLOR_RED;
  }

  if (b > r * 1.12f && b > g * 1.05f) {
    return COLOR_BLUE;
  }

  if (spread < 1.35f) {
    return COLOR_WHITE;
  }

  return COLOR_NONE;
}

const char* colorName(BallColor color) {
  switch (color) {
    case COLOR_RED:
      return "RED";
    case COLOR_WHITE:
      return "WHITE";
    case COLOR_BLUE:
      return "BLUE";
    default:
      return "NONE";
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
      return SORT_HOME_ANGLE;
  }
}

void pulseSortGate() {
  sortGateServo.write(SORT_GATE_OPEN);
  delay(SORT_GATE_OPEN_MS);
  sortGateServo.write(SORT_GATE_CLOSED);
  delay(250);
}

void sortOneBall(BallColor color) {
  int targetAngle = sortAngleForColor(color);

  Serial.print("Sorting: ");
  Serial.println(colorName(color));

  sortServo.write(targetAngle);
  delay(SORT_SETTLE_MS);

  pulseSortGate();

  sortServo.write(SORT_HOME_ANGLE);
  delay(200);
}

void dumpStorageGate(Servo &gateServo, int openAngle, int closedAngle, const char* label) {
  Serial.print("Dumping ");
  Serial.println(label);

  gateServo.write(openAngle);
  delay(STORE_GATE_OPEN_MS);
  gateServo.write(closedAngle);
}

void readAndSort() {
  if (millis() - lastReadTime < SENSOR_READ_DELAY_MS) {
    return;
  }

  lastReadTime = millis();

  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  if (!ballLatched && c >= BALL_PRESENT_CLEAR) {
    BallColor color = classifyColor(r, g, b, c);

    Serial.print("R: ");
    Serial.print(r);
    Serial.print("  G: ");
    Serial.print(g);
    Serial.print("  B: ");
    Serial.print(b);
    Serial.print("  C: ");
    Serial.print(c);
    Serial.print("  ->  ");
    Serial.println(colorName(color));

    if (color != COLOR_NONE) {
      sortOneBall(color);
    }

    ballLatched = true;
  }

  if (ballLatched && c <= BALL_GONE_CLEAR) {
    ballLatched = false;
  }
}

void runDumpSequence() {
  sortServo.write(SORT_HOME_ANGLE);
  sortGateServo.write(SORT_GATE_CLOSED);
  delay(500);

  dumpStorageGate(redStoreServo, RED_STORE_OPEN, RED_STORE_CLOSED, "RED STORAGE");
  delay(STORAGE_INTERVAL_MS);

  dumpStorageGate(whiteStoreServo, WHITE_STORE_OPEN, WHITE_STORE_CLOSED, "WHITE STORAGE");
  delay(STORAGE_INTERVAL_MS);

  dumpStorageGate(blueStoreServo, BLUE_STORE_OPEN, BLUE_STORE_CLOSED, "BLUE STORAGE");

  dumpPhaseDone = true;
  Serial.println("Dump sequence complete");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  sortServo.attach(PIN_SORT_SERVO);
  sortGateServo.attach(PIN_SORT_GATE);
  redStoreServo.attach(PIN_RED_STORE);
  whiteStoreServo.attach(PIN_WHITE_STORE);
  blueStoreServo.attach(PIN_BLUE_STORE);

  sortServo.write(SORT_HOME_ANGLE);
  sortGateServo.write(SORT_GATE_CLOSED);
  redStoreServo.write(RED_STORE_CLOSED);
  whiteStoreServo.write(WHITE_STORE_CLOSED);
  blueStoreServo.write(BLUE_STORE_CLOSED);

  if (!tcs.begin()) {
    Serial.println("TCS34725 not found. Check wiring.");
    while (true) {
    }
  }

  Serial.println("Starting 30 second sorting test");
  sortStartTime = millis();
}

void loop() {
  if (!sortPhaseDone) {
    if (millis() - sortStartTime < SORT_TIME_MS) {
      readAndSort();
      return;
    } else {
      sortPhaseDone = true;
      Serial.println("Sorting time ended. Starting storage dump sequence");
    }
  }

  if (!dumpPhaseDone) {
    runDumpSequence();
  }
}
