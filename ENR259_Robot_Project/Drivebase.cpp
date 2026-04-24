#include "Config.h"
#include "Drivebase.h"
#include <Wire.h>
#include <math.h>

namespace {

enum DrivebaseState {
  DB_IDLE,
  DB_STRAIGHT,
  DB_PIVOT_LEFT,
  DB_PIVOT_RIGHT,
  DB_BRAKE,
  DB_SETTLE
};

// Encoders disabled — ISRs kept but not attached
volatile long leftCounts  = 0;
volatile long rightCounts = 0;

float gyroZOffset  = 0.0f;
float yawAngle     = 0.0f;
float lastGyroZRate = 0.0f;
unsigned long lastGyroTime = 0;

float pidIntegral  = 0.0f;
float pidLastError = 0.0f;

DrivebaseState drivebaseState      = DB_IDLE;
unsigned long  drivebaseStateStart = 0;

float         activeTargetHeading    = 0.0f;
int           activeBaseSpeed        = 0;
int           activeDirection        = 1;
unsigned long activeTargetDurationMs = 0;
TurnDirection activeTurnDirection    = TURN_LEFT_DIR;
float         activeTurnStartYaw     = 0.0f;
float         activeTurnMagnitude    = 0.0f;

struct DriveLogSample {
  uint32_t ms;
  uint32_t moveMs;
  uint16_t sampleDtMs;
  uint16_t loopDtMs;
  uint8_t state;
  int8_t direction;
  int16_t baseSpeed;
  int16_t target10;
  int16_t yaw10;
  int16_t error10;
  int16_t p10;
  int16_t i10;
  int16_t d10;
  int16_t rawCorrection10;
  int16_t correction10;
  int16_t leftSpeed;
  int16_t rightSpeed;
  int16_t gyroZ10;
  uint8_t flags;
};

constexpr uint8_t LOG_FLAG_CORRECTION_CLAMPED = 0x01;

DriveLogSample driveLog[DRIVE_LOG_MAX_SAMPLES];
int driveLogCount = 0;
uint16_t driveLogRunId = 0;
bool driveLogOverflowed = false;
unsigned long driveLogLastSampleMs = 0;
unsigned long driveLogLastUpdateMs = 0;

void leftEncoderISR() {
  if (digitalRead(2) == HIGH) leftCounts++;
  else                         leftCounts--;
}

void rightEncoderISR() {
  if (digitalRead(3) == HIGH) rightCounts++;
  else                         rightCounts--;
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 2, true);
  if (Wire.available() < 2) return 0.0f;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return (raw / 131.0f) - gyroZOffset;
}

void calibrateGyro() {
  float sum = 0.0f;
  int validSamples = 0;

  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU_ADDR, 2, true);
    if (Wire.available() >= 2) {
      int16_t raw = (Wire.read() << 8) | Wire.read();
      sum += raw / 131.0f;
      validSamples++;
    }
    delay(2);
  }

  if (validSamples > 0) {
    gyroZOffset = sum / validSamples;
  } else {
    gyroZOffset = 0.0f;
  }

  if (DEBUG_DRIVE) {
    Serial.print("Gyro calibrated. validSamples=");
    Serial.print(validSamples);
    Serial.print(" offset=");
    Serial.println(gyroZOffset, 6);
  }
}

void initMPU() {
  Wire.begin();
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00);
  mpuWrite(0x1B, 0x00);
  mpuWrite(0x1A, 0x03);
  delay(100);
  calibrateGyro();
  yawAngle     = 0.0f;
  lastGyroTime = micros();
}

void updateYaw() {
  unsigned long now = micros();
  if (lastGyroTime == 0) {
    lastGyroTime = now;
    return;
  }

  float dt = (now - lastGyroTime) / 1000000.0f;
  lastGyroTime = now;
  lastGyroZRate = readGyroZ();
  yawAngle += (GYRO_Z_SIGN * lastGyroZRate) * dt;
}

void resetPID() {
  pidIntegral  = 0.0f;
  pidLastError = 0.0f;
}

void setMotorRaw(int pwmPin, int in1, int in2, int speed, bool invert) {
  speed = constrain(speed, -255, 255);
  if (invert) speed = -speed;

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void drive(int leftSpeed, int rightSpeed) {
  digitalWrite(STBY, HIGH);
  setMotorRaw(PWMA, AIN1, AIN2, leftSpeed,  INVERT_A);
  setMotorRaw(PWMB, BIN1, BIN2, rightSpeed, INVERT_B);
}

void setDrivebaseState(DrivebaseState nextState) {
  drivebaseState      = nextState;
  drivebaseStateStart = millis();
  if (DEBUG_DRIVE) {
    Serial.print("Drivebase state -> ");
    Serial.println(static_cast<int>(drivebaseState));
  }
}

int16_t scaled10(float value) {
  long scaled = lroundf(value * 10.0f);
  scaled = constrain(scaled, -32768L, 32767L);
  return static_cast<int16_t>(scaled);
}

uint16_t clampU16(unsigned long value) {
  if (value > 65535UL) return 65535;
  return static_cast<uint16_t>(value);
}

void logDriveSample(unsigned long now,
                    unsigned long elapsed,
                    unsigned long loopDtMs,
                    float error,
                    float pTerm,
                    float iTerm,
                    float dTerm,
                    float rawCorrection,
                    float correction,
                    int leftSpeed,
                    int rightSpeed) {
  if (!ENABLE_DRIVE_LOG) return;

  if (driveLogLastSampleMs != 0 &&
      now - driveLogLastSampleMs < DRIVE_LOG_SAMPLE_MS) {
    return;
  }

  unsigned long sampleDtMs =
    driveLogLastSampleMs == 0 ? 0 : now - driveLogLastSampleMs;
  driveLogLastSampleMs = now;

  if (driveLogCount >= DRIVE_LOG_MAX_SAMPLES) {
    driveLogOverflowed = true;
    return;
  }

  uint8_t flags = 0;
  if (fabsf(rawCorrection - correction) > 0.001f) {
    flags |= LOG_FLAG_CORRECTION_CLAMPED;
  }

  DriveLogSample &sample = driveLog[driveLogCount++];
  sample.ms              = now;
  sample.moveMs          = elapsed;
  sample.sampleDtMs      = clampU16(sampleDtMs);
  sample.loopDtMs        = clampU16(loopDtMs);
  sample.state           = static_cast<uint8_t>(drivebaseState);
  sample.direction       = static_cast<int8_t>(activeDirection);
  sample.baseSpeed       = static_cast<int16_t>(activeBaseSpeed);
  sample.target10        = scaled10(activeTargetHeading);
  sample.yaw10           = scaled10(yawAngle);
  sample.error10         = scaled10(error);
  sample.p10             = scaled10(pTerm);
  sample.i10             = scaled10(iTerm);
  sample.d10             = scaled10(dTerm);
  sample.rawCorrection10 = scaled10(rawCorrection);
  sample.correction10    = scaled10(correction);
  sample.leftSpeed       = static_cast<int16_t>(leftSpeed);
  sample.rightSpeed      = static_cast<int16_t>(rightSpeed);
  sample.gyroZ10         = scaled10(lastGyroZRate);
  sample.flags           = flags;
}

}  // namespace

void initDrivebase() {
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  analogWriteResolution(8);
  analogWriteFrequency(PWMA, 20000);
  analogWriteFrequency(PWMB, 20000);

  // Encoders disabled — uncomment when re-enabling:
  // pinMode(LEFT_ENC_A,  INPUT_PULLUP);
  // pinMode(LEFT_ENC_B,  INPUT_PULLUP);
  // pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  // pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);

  stopDrivebase();
  initMPU();
}

void stopDrivebase() {
  drive(0, 0);
}

void resetDriveLog() {
  driveLogCount = 0;
  driveLogRunId++;
  driveLogOverflowed = false;
  driveLogLastSampleMs = 0;
  driveLogLastUpdateMs = 0;
}

void printDriveLog() {
  if (!ENABLE_DRIVE_LOG) return;

  Serial.print("DRIVE_LOG_BEGIN,run=");
  Serial.print(driveLogRunId);
  Serial.print(",samples=");
  Serial.print(driveLogCount);
  Serial.print(",overflow=");
  Serial.println(driveLogOverflowed ? 1 : 0);

  Serial.println("ms,moveMs,sampleDtMs,loopDtMs,state,baseSpeed,direction,target,yaw,error,p,i,d,rawCorrection,correction,leftSpeed,rightSpeed,gyroZ,flags");

  for (int i = 0; i < driveLogCount; i++) {
    const DriveLogSample &sample = driveLog[i];

    Serial.print(sample.ms);
    Serial.print(",");
    Serial.print(sample.moveMs);
    Serial.print(",");
    Serial.print(sample.sampleDtMs);
    Serial.print(",");
    Serial.print(sample.loopDtMs);
    Serial.print(",");
    Serial.print(static_cast<int>(sample.state));
    Serial.print(",");
    Serial.print(sample.baseSpeed);
    Serial.print(",");
    Serial.print(static_cast<int>(sample.direction));
    Serial.print(",");
    Serial.print(sample.target10 / 10.0f, 1);
    Serial.print(",");
    Serial.print(sample.yaw10 / 10.0f, 1);
    Serial.print(",");
    Serial.print(sample.error10 / 10.0f, 1);
    Serial.print(",");
    Serial.print(sample.p10 / 10.0f, 1);
    Serial.print(",");
    Serial.print(sample.i10 / 10.0f, 1);
    Serial.print(",");
    Serial.print(sample.d10 / 10.0f, 1);
    Serial.print(",");
    Serial.print(sample.rawCorrection10 / 10.0f, 1);
    Serial.print(",");
    Serial.print(sample.correction10 / 10.0f, 1);
    Serial.print(",");
    Serial.print(sample.leftSpeed);
    Serial.print(",");
    Serial.print(sample.rightSpeed);
    Serial.print(",");
    Serial.print(sample.gyroZ10 / 10.0f, 1);
    Serial.print(",");
    Serial.println(static_cast<int>(sample.flags));
  }

  Serial.println("DRIVE_LOG_END");
}

void resetYaw() {
  yawAngle     = 0.0f;
  lastGyroTime = micros();
  pidIntegral  = 0.0f;
  pidLastError = 0.0f;

  if (DEBUG_DRIVE) {
    Serial.println("Yaw reset to 0.");
  }
}

void startStraightMove(unsigned long durationMs, int baseSpeed, int direction, float targetHeading) {
  resetPID();
  resetDriveLog();
  activeTargetDurationMs = durationMs;
  activeBaseSpeed        = baseSpeed;
  activeDirection        = direction;
  activeTargetHeading    = targetHeading;

  if (DEBUG_DRIVE) {
    Serial.print("Start straight. durationMs=");
    Serial.print(activeTargetDurationMs);
    Serial.print(" heading=");
    Serial.println(activeTargetHeading);
  }

  setDrivebaseState(DB_STRAIGHT);
}

void startPivotToHeading(float targetHeading, TurnDirection direction) {
  resetPID();
  activeTargetHeading = targetHeading;
  activeTurnDirection = direction;
  activeTurnStartYaw  = yawAngle;
  activeTurnMagnitude = fabsf(activeTargetHeading - activeTurnStartYaw);

  if (DEBUG_DRIVE) {
    Serial.print("Start pivot. heading=");
    Serial.print(activeTargetHeading);
    Serial.print(" dir=");
    Serial.print(direction == TURN_LEFT_DIR ? "left" : "right");
    Serial.print(" startYaw=");
    Serial.print(activeTurnStartYaw);
    Serial.print(" targetDelta=");
    Serial.println(activeTurnMagnitude);
  }

  if (direction == TURN_LEFT_DIR) {
    setDrivebaseState(DB_PIVOT_LEFT);
  } else {
    setDrivebaseState(DB_PIVOT_RIGHT);
  }
}

bool drivebaseIsBusy() {
  return drivebaseState != DB_IDLE;
}

float getYawAngle() {
  return yawAngle;
}

void updateDrivebase() {
  updateYaw();

  if (drivebaseState == DB_IDLE) return;

  unsigned long now     = millis();
  unsigned long elapsed = now - drivebaseStateStart;
  unsigned long loopDtMs = 0;

  if (ENABLE_DRIVE_LOG && drivebaseState == DB_STRAIGHT) {
    loopDtMs = driveLogLastUpdateMs == 0 ? 0 : now - driveLogLastUpdateMs;
    driveLogLastUpdateMs = now;
  }

  switch (drivebaseState) {
    case DB_STRAIGHT: {
      if (elapsed >= activeTargetDurationMs) {
        stopDrivebase();
        if (DRIVE_LOG_AUTO_PRINT) {
          printDriveLog();
        }
        setDrivebaseState(DB_SETTLE);
        break;
      }

      float error      = yawAngle - activeTargetHeading;
      pidIntegral     += error * 0.01f;
      pidIntegral      = constrain(pidIntegral, -30.0f, 30.0f);
      float derivative = error - pidLastError;
      pidLastError     = error;

      float pTerm = KP * error;
      float iTerm = KI * pidIntegral;
      float dTerm = KD * derivative;
      float rawCorrection = pTerm + iTerm + dTerm;
      float correction = rawCorrection;
      correction = constrain(correction, -50.0f, 50.0f);

      int leftSpeed  = constrain(activeDirection * activeBaseSpeed + static_cast<int>(correction), -255, 255);
      int rightSpeed = constrain(activeDirection * activeBaseSpeed - static_cast<int>(correction), -255, 255);

      logDriveSample(now,
                     elapsed,
                     loopDtMs,
                     error,
                     pTerm,
                     iTerm,
                     dTerm,
                     rawCorrection,
                     correction,
                     leftSpeed,
                     rightSpeed);

      drive(leftSpeed, rightSpeed);
      break;
    }

    case DB_PIVOT_LEFT: {
      float turned    = fabsf(yawAngle - activeTurnStartYaw);
      float remaining = activeTurnMagnitude - turned;

      if (remaining <= TURN_STOP_TOLERANCE_DEG || elapsed >= TURN_TIMEOUT_MS) {
        yawAngle = activeTargetHeading;
        drive(TURN_SPEED, -TURN_SPEED);
        setDrivebaseState(DB_BRAKE);
        break;
      }

      int turnSpd = constrain(static_cast<int>(remaining * 3.0f), 80, TURN_SPEED);
      drive(-turnSpd, turnSpd);
      break;
    }

    case DB_PIVOT_RIGHT: {
      float turned    = fabsf(yawAngle - activeTurnStartYaw);
      float remaining = activeTurnMagnitude - turned;

      if (remaining <= TURN_STOP_TOLERANCE_DEG || elapsed >= TURN_TIMEOUT_MS) {
        yawAngle = activeTargetHeading;
        drive(-TURN_SPEED, TURN_SPEED);
        setDrivebaseState(DB_BRAKE);
        break;
      }

      int turnSpd = constrain(static_cast<int>(remaining * 3.0f), 80, TURN_SPEED);
      drive(turnSpd, -turnSpd);
      break;
    }

    case DB_BRAKE:
      if (elapsed >= TURN_BRAKE_MS) {
        stopDrivebase();
        setDrivebaseState(DB_SETTLE);
      }
      break;

    case DB_SETTLE:
      stopDrivebase();
      if (elapsed >= PAUSE_MS) {
        setDrivebaseState(DB_IDLE);
      }
      break;

    case DB_IDLE:
    default:
      break;
  }
}
