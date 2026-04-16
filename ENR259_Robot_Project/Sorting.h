#pragma once

enum DetectedColor {
  NONE,
  RED,
  WHITE,
  BLUE
};

void initSorting();
void updateSorting();
DetectedColor detectColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
