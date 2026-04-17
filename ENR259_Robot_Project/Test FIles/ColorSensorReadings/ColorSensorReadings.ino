#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs =
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const unsigned long READ_INTERVAL_MS = 100UL;
unsigned long lastReadTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!tcs.begin()) {
    Serial.println("TCS34725 not found. Check wiring.");
    while (true) {
    }
  }

  Serial.println("Ambient read mode started");
  Serial.println("Printing R, G, B, C every 100 ms");
  Serial.println();
}

void loop() {
  if (millis() - lastReadTime >= READ_INTERVAL_MS) {
    lastReadTime = millis();

    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    Serial.print("R: ");
    Serial.print(r);
    Serial.print("  G: ");
    Serial.print(g);
    Serial.print("  B: ");
    Serial.print(b);
    Serial.print("  C: ");
    Serial.println(c);
  }
}