#include "Config.h"
#include "Drivebase.h"
#include "Gates.h"
#include "Mission.h"
#include "Scooper.h"
#include "Sorting.h"
#include "Conveyor.h"

namespace {

enum MissionState {
  MISSION_START_LANE,
  MISSION_WAIT_LANE,
  MISSION_WAIT_SCOOPER,
  MISSION_WAIT_TURN_TO_SHIFT,
  MISSION_WAIT_SHIFT,
  MISSION_WAIT_TURN_TO_NEXT_LANE,
  MISSION_WAIT_FINAL_SIDEWAYS_TURN,
  MISSION_WAIT_RED_DROP,
  MISSION_WAIT_MOVE_TO_WHITE,
  MISSION_WAIT_WHITE_DROP,
  MISSION_WAIT_MOVE_TO_BLUE,
  MISSION_WAIT_BLUE_DROP,
  MISSION_DONE
};

MissionState missionState = MISSION_START_LANE;
int currentLane = 1;

float headingForLane(int lane) {
  switch (lane) {
    case 1: return LANE_1_HEADING;
    case 2: return LANE_2_HEADING;
    case 3: return LANE_3_HEADING;
    case 4: return LANE_4_HEADING;
    case 5:
    default: return LANE_5_HEADING;
  }
}

void setMissionState(MissionState nextState) {
  missionState = nextState;
  if (DEBUG_MISSION) {
    Serial.print("Mission state -> ");
    Serial.println(static_cast<int>(missionState));
  }
}

}  // namespace

void initMission() {
  currentLane = 1;
  setMissionState(MISSION_START_LANE);
}

bool missionIsComplete() {
  return missionState == MISSION_DONE;
}

void updateMission() {
  switch (missionState) {
    case MISSION_START_LANE:
      if (!drivebaseIsBusy()) {
        conveyorOn();
        setScooperDown();
        if (currentLane % 2 == 1) {
          startStraightMove(ODD_LANE_DRIVE_MS, DRIVE_SPEED, +1, headingForLane(currentLane));
        }
        else {
          startStraightMove(EVEN_LANE_DRIVE_MS, DRIVE_SPEED, +1, headingForLane(currentLane));
        }
        setMissionState(MISSION_WAIT_LANE);
      }
      break;

    case MISSION_WAIT_LANE:
      if (!drivebaseIsBusy()) {
        conveyorOff();

        // Wait for the sorter to finish any active mechanical cycle
        // before lifting the scooper or starting a pivot.
        if (!sortingIsBusy()) {
          startScooperCycle();
          setMissionState(MISSION_WAIT_SCOOPER);
        }
      }
      break;

    case MISSION_WAIT_SCOOPER:
      if (!scooperIsBusy()) {
        setScooperUp();
        if (currentLane < TOTAL_LANES) {
          if (currentLane % 2 == 1) {
            startPivotToHeading(SHIFT_HEADING, TURN_LEFT_DIR);
          } else {
            startPivotToHeading(SHIFT_HEADING, TURN_RIGHT_DIR);
          }
          setMissionState(MISSION_WAIT_TURN_TO_SHIFT);
        } else {
          startPivotToHeading(FINAL_WALL_HEADING, TURN_LEFT_DIR);
          setMissionState(MISSION_WAIT_FINAL_SIDEWAYS_TURN);
        }
      }
      break;

    case MISSION_WAIT_TURN_TO_SHIFT:
      if (!drivebaseIsBusy()) {
        startStraightMove(LANE_SHIFT_MS, DRIVE_SPEED, +1, SHIFT_HEADING);
        setMissionState(MISSION_WAIT_SHIFT);
      }
      break;

    case MISSION_WAIT_SHIFT:
      if (!drivebaseIsBusy()) {
        if (currentLane % 2 == 1) {
          startPivotToHeading(180.0f, TURN_LEFT_DIR);
        } else {
          startPivotToHeading(0.0f, TURN_RIGHT_DIR);
        }
        setMissionState(MISSION_WAIT_TURN_TO_NEXT_LANE);
      }
      break;

    case MISSION_WAIT_TURN_TO_NEXT_LANE:
      if (!drivebaseIsBusy()) {
        currentLane++;
        setMissionState(MISSION_START_LANE);
      }
      break;

    case MISSION_WAIT_FINAL_SIDEWAYS_TURN:
      if (!drivebaseIsBusy()) {
        startGateCycle(GATE_RED);
        setMissionState(MISSION_WAIT_RED_DROP);
      }
      break;

    case MISSION_WAIT_RED_DROP:
      if (!gatesAreBusy()) {
        startStraightMove(FINAL_DROP_STEP_MS, DRIVE_SPEED, +1, FINAL_WALL_HEADING);
        setMissionState(MISSION_WAIT_MOVE_TO_WHITE);
      }
      break;

    case MISSION_WAIT_MOVE_TO_WHITE:
      if (!drivebaseIsBusy()) {
        startGateCycle(GATE_WHITE);
        setMissionState(MISSION_WAIT_WHITE_DROP);
      }
      break;

    case MISSION_WAIT_WHITE_DROP:
      if (!gatesAreBusy()) {
        startStraightMove(FINAL_DROP_STEP_MS, DRIVE_SPEED, +1, FINAL_WALL_HEADING);
        setMissionState(MISSION_WAIT_MOVE_TO_BLUE);
      }
      break;

    case MISSION_WAIT_MOVE_TO_BLUE:
      if (!drivebaseIsBusy()) {
        startGateCycle(GATE_BLUE);
        setMissionState(MISSION_WAIT_BLUE_DROP);
      }
      break;

    case MISSION_WAIT_BLUE_DROP:
      if (!gatesAreBusy()) {
        stopDrivebase();
        setScooperUp();
        setMissionState(MISSION_DONE);
      }
      break;

    case MISSION_DONE:
      stopDrivebase();
      conveyorOff();
      break;
  }
}
