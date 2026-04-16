#pragma once

enum GateColor {
  GATE_RED,
  GATE_WHITE,
  GATE_BLUE
};

void initGates();
void updateGates();
void startGateCycle(GateColor gateColor);
bool gatesAreBusy();
