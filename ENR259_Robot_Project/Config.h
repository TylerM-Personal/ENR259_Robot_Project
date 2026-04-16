#pragma once
#include <Arduino.h>

// Synced pin map from updated_integrated_code_fixed(1).zip

// =========================
// Debug toggles
// =========================
constexpr bool DEBUG_GENERAL = true;
constexpr bool DEBUG_DRIVE = false;
constexpr bool DEBUG_SORTING = false;
constexpr bool DEBUG_SCOOPER = false;
constexpr bool DEBUG_GATES = false;
constexpr bool DEBUG_MISSION = true;

// =========================
// Module enable toggles
// =========================
constexpr bool ENABLE_SORTING = true;

// =========================
// Drivebase pins (Teensy 4.0)
// Preserved from message (4)
// =========================
constexpr int PWMA = 14;
constexpr int AIN1 = 16;
constexpr int AIN2 = 15;

constexpr int PWMB = 8;
constexpr int BIN1 = 6;
constexpr int BIN2 = 7;

constexpr int STBY = 9;

// =========================
// Encoder pins
// Preserved from message (4)
// =========================
constexpr int LEFT_ENC_A = 2;
constexpr int LEFT_ENC_B = 4;
constexpr int RIGHT_ENC_A = 1;
constexpr int RIGHT_ENC_B = 3;

// =========================
// I2C / sensors
// =========================
constexpr uint8_t MPU_ADDR = 0x68;

// =========================
// Servo pins
// Keep off pins 18 and 19 because I2C is in use.
// Final drop gates are separate from sorting mechanism servos.
// =========================
constexpr int SCOOPER_SERVO_PIN = 10;
constexpr int RED_GATE_SERVO_PIN = 11;
constexpr int WHITE_GATE_SERVO_PIN = 12;
constexpr int BLUE_GATE_SERVO_PIN = 13;

constexpr int SORTING_SERVO_PIN = 5;
constexpr int SORT_GATE_SERVO_PIN = 17;

// =========================
// Field / path constants
// Preserved from message (4)
// =========================
constexpr float LANE_LENGTH_MM = 1500.0f;
constexpr float LANE_WIDTH_MM = 170.0f;
constexpr int TOTAL_LANES = 5;
constexpr float FINAL_DROP_STEP_MM = 457.2f;  // 1.5 ft

constexpr float LANE_1_HEADING = 0.0f;
constexpr float LANE_2_HEADING = 180.0f;
constexpr float LANE_3_HEADING = 0.0f;
constexpr float LANE_4_HEADING = 180.0f;
constexpr float LANE_5_HEADING = 0.0f;
constexpr float SHIFT_HEADING = 90.0f;
constexpr float FINAL_WALL_HEADING = 90.0f;

// =========================
// Drivebase tuning
// Preserved from message (4)
// =========================
constexpr bool INVERT_A = false;
constexpr bool INVERT_B = false;

constexpr int DRIVE_SPEED = 170;
constexpr int TURN_SPEED = 140;
constexpr unsigned long PAUSE_MS = 200;
constexpr unsigned long TURN_BRAKE_MS = 60;
constexpr unsigned long TURN_TIMEOUT_MS = 2500;
constexpr float TURN_STOP_TOLERANCE_DEG = 2.0f;
constexpr float GYRO_Z_SIGN = -1.0f;

constexpr float KP = 1.0f;
constexpr float KI = 0.0f;
constexpr float KD = 0.4f;

constexpr float DIST_PER_COUNT = 0.505f;

// =========================
// Scooper positions
// Preserved from message (4)
// =========================
constexpr int SCOOPER_START_POS = 160;
constexpr int SCOOPER_DOWN_POS = 160;
constexpr int SCOOPER_UP_POS = 20;
constexpr unsigned long SCOOPER_DOWN_HOLD_MS = 1000;
constexpr unsigned long SCOOPER_UP_HOLD_MS = 1000;

// =========================
// Final 3 top-wall gates
// User requested 0 = closed and 90 = open
// =========================
constexpr int GATE_CLOSED_POS = 0;
constexpr int GATE_OPEN_POS = 90;
constexpr unsigned long GATE_OPEN_HOLD_MS = 650;
constexpr unsigned long GATE_SETTLE_MS = 250;

// =========================
// Sorting mechanism tuning
// This uses the same detection thresholds and same motion sequence as
// your standalone Storting.ino, but inside the safer non blocking
// project state machine used by this integrated build.
//
// These four angle values are the ones to change if your hardware needs
// different calibrated positions.
// =========================
constexpr int SORT_GATE_CLOSED_POS = 90;
constexpr int SORT_GATE_OPEN_POS = 10;

constexpr int SORT_ORIGIN_POS = 125;
constexpr int SORT_RED_POS = 70;
constexpr int SORT_WHITE_POS = 125;
constexpr int SORT_BLUE_POS = 175;

constexpr unsigned long SORTING_SERVO_DELAY = 250;
constexpr unsigned long SORT_GATE_OPEN_TIME = 125;
constexpr unsigned long SORT_RESET_DELAY = 150;
constexpr unsigned long SORT_SENSOR_PRINT_MS = 200;

// =========================
// Conveyor motor (MOSFET PWM)
// =========================
constexpr int CONVEYOR_PIN = 22;   // pick any free PWM-capable pin
constexpr int CONVEYOR_SPEED = 100; // 0–255
