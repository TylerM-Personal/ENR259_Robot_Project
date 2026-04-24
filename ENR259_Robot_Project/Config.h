#pragma once
#include <Arduino.h>

// =========================
// Debug toggles
// =========================
constexpr bool DEBUG_GENERAL  = true;
constexpr bool DEBUG_DRIVE    = false;
constexpr bool DEBUG_SORTING  = false;
constexpr bool DEBUG_SCOOPER  = false;
constexpr bool DEBUG_GATES    = false;
constexpr bool DEBUG_MISSION  = true;

// =========================
// Module enable toggles
// =========================
constexpr bool ENABLE_SORTING  = true;    // enabled — uses top sorter + TCS34725
constexpr bool ENABLE_ENCODERS = false;   // still disabled — sorter now uses pins 2 and 3, scooper uses 4 and 5

// =========================
// Drivebase pins (Teensy 4.0)
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
// Keep disabled in this version.
// Sorter now uses pins 2 and 3, and scooper uses pins 4 and 5.
// =========================
// constexpr int LEFT_ENC_A  = 2;
// constexpr int LEFT_ENC_B  = 4;
// constexpr int RIGHT_ENC_A = 1;
// constexpr int RIGHT_ENC_B = 3;

// =========================
// I2C / sensors
// MPU-6050 + TCS34725 share the same I2C bus on pins 18 and 19.
// Do not use 18 or 19 for servos or other signals.
// =========================
constexpr uint8_t MPU_ADDR = 0x68;

// =========================
// Scooper servo pins — two mirrored MG996R
// =========================
constexpr int SCOOPER_SERVO_LEFT_PIN  = 4;
constexpr int SCOOPER_SERVO_RIGHT_PIN = 5;

// =========================
// Final wall gate servo pins
// =========================
constexpr int RED_GATE_SERVO_PIN   = 11;
constexpr int WHITE_GATE_SERVO_PIN = 12;
constexpr int BLUE_GATE_SERVO_PIN  = 13;

// =========================
// Top sorting mechanism pins
// =========================
constexpr int SORTING_SERVO_PIN   = 3;
constexpr int SORT_GATE_SERVO_PIN = 2;

// =========================
// Field / path constants — TIME BASED
// Tune all _MS values on your actual turf
// =========================
constexpr int TOTAL_LANES = 5;

constexpr unsigned long LANE_DRIVE_MS      = 4200;
constexpr unsigned long LANE_SHIFT_MS      = 800;
constexpr unsigned long FINAL_DROP_STEP_MS = 1200;

// Heading constants — used for gyro PID straight correction
constexpr float LANE_1_HEADING     = 0.0f;
constexpr float LANE_2_HEADING     = 180.0f;
constexpr float LANE_3_HEADING     = 0.0f;
constexpr float LANE_4_HEADING     = 180.0f;
constexpr float LANE_5_HEADING     = 0.0f;
constexpr float SHIFT_HEADING      = 90.0f;
constexpr float FINAL_WALL_HEADING = 90.0f;

// =========================
// Drivebase tuning
// =========================
constexpr bool INVERT_A = false;
constexpr bool INVERT_B = false;

constexpr int DRIVE_SPEED = 170;
constexpr int TURN_SPEED  = 140;
constexpr unsigned long PAUSE_MS        = 200;
constexpr unsigned long TURN_BRAKE_MS   = 60;
constexpr unsigned long TURN_TIMEOUT_MS = 2500;
constexpr float TURN_STOP_TOLERANCE_DEG = 2.0f;
constexpr float GYRO_Z_SIGN             = -1.0f;

constexpr float KP = 1.0f;
constexpr float KI = 0.0f;
constexpr float KD = 0.4f;

constexpr float DIST_PER_COUNT = 0.505f;

// =========================
// Drivebase logging
// Logs into RAM while driving, then prints CSV after each straight move.
// =========================
constexpr bool ENABLE_DRIVE_LOG     = true;
constexpr bool DRIVE_LOG_AUTO_PRINT = true;
constexpr unsigned long DRIVE_LOG_SAMPLE_MS = 50;
constexpr int DRIVE_LOG_MAX_SAMPLES = 160;

// =========================
// Scooper positions — two mirrored MG996R
// =========================
constexpr int SCOOPER_LEFT_DOWN_POS  = 150;
constexpr int SCOOPER_RIGHT_DOWN_POS = 30;
constexpr int SCOOPER_LEFT_UP_POS    = 30;
constexpr int SCOOPER_RIGHT_UP_POS   = 150;
constexpr unsigned long SCOOPER_DOWN_HOLD_MS = 1000;
constexpr unsigned long SCOOPER_UP_HOLD_MS   = 1000;

// =========================
// Final 3 top-wall gates
// =========================
constexpr int GATE_CLOSED_POS = 0;
constexpr int GATE_OPEN_POS   = 90;
constexpr unsigned long GATE_OPEN_HOLD_MS = 650;
constexpr unsigned long GATE_SETTLE_MS    = 250;

// =========================
// Sorting mechanism tuning
// Updated to match latest tested standalone Teensy sorting code
// =========================
constexpr int SORT_GATE_CLOSED_POS = 130;
constexpr int SORT_GATE_OPEN_POS   = 45;

constexpr int SORT_ORIGIN_POS = 90;
constexpr int SORT_RED_POS    = 50;
constexpr int SORT_WHITE_POS  = 90;
constexpr int SORT_BLUE_POS   = 180;

constexpr unsigned long SORT_AMBIENT_CAL_MS  = 2000;
constexpr unsigned long SORTING_SERVO_DELAY  = 300;
constexpr unsigned long SORT_GATE_OPEN_TIME  = 1400;
constexpr unsigned long SORT_RESET_DELAY     = 200;
constexpr unsigned long SORT_SENSOR_READ_MS  = 60;
constexpr unsigned long SORT_SAMPLE_GAP_MS   = 12;
constexpr unsigned long SORT_SENSOR_PRINT_MS = 200;
constexpr int           SORT_SAMPLE_COUNT    = 5;

// =========================
// Conveyor motor (MOSFET PWM)
// =========================
constexpr int CONVEYOR_PIN   = 22;
constexpr int CONVEYOR_SPEED = 150;
