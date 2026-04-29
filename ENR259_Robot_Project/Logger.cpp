#include "Logger.h"
#include <Arduino.h>

namespace {
  unsigned long lastLogTime = 0;
  constexpr unsigned long LOG_INTERVAL_MS = 200;
}

void initLogger() {
  Serial1.begin(9600);
  Serial1.println("Time_ms | R     | G     | B     | C     | G*1.2  | B*1.83");
  Serial1.println("--------|-------|-------|-------|-------|--------|--------");
}

void logColorData(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  unsigned long now = millis();
  if (now - lastLogTime < LOG_INTERVAL_MS) return;
  lastLogTime = now;

  char buf[32];

  sprintf(buf, "%7lu", now);
  Serial1.print(buf);
  Serial1.print(" | ");

  sprintf(buf, "%5u", r);
  Serial1.print(buf);
  Serial1.print(" | ");

  sprintf(buf, "%5u", g);
  Serial1.print(buf);
  Serial1.print(" | ");

  sprintf(buf, "%5u", b);
  Serial1.print(buf);
  Serial1.print(" | ");

  sprintf(buf, "%5u", c);
  Serial1.print(buf);
  Serial1.print(" | ");

  sprintf(buf, "%6.1f", g * 1.2f);
  Serial1.print(buf);
  Serial1.print(" | ");

  sprintf(buf, "%6.1f", b * 1.83f);
  Serial1.println(buf);
}
