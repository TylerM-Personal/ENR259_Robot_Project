#pragma once

void initLimitSwitches();
void updateLimitSwitches();
void clearLimitSwitchClicks();

bool firstLimitPressed();
bool backLimitPressed();

// Returns true once per clean press, then clears itself.
bool firstLimitClicked();
bool backLimitClicked();
