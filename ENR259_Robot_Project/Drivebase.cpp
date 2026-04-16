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

volatile long leftCounts = 0;
volatile long rightCounts = 0;

float gyroZOffset = 0.0f;
float yawAngle = 0.0f;
unsigned long lastGyroTime = 0;

float pidIntegral = 0.0f;
float pidLastError = 0.0f;

DrivebaseState drivebaseState = DB_IDLE;
unsigned long drivebaseStateStart = 0;

float activeTargetHeading = 0.0f;
int activeBaseSpeed = 0;
int activeDirection = 1;
long activeTargetCounts = 0;
TurnDirection activeTurnDirection = TURN_LEFT_DIR;
float activeTurnStartYaw = 0.0f;
float activeTurnMagnitude = 0.0f;

void leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    leftCounts++;
  } else {
    leftCounts--;
  }
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    rightCounts++;
  } else {
    rightCounts--;
  }
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

  if (Wire.available() < 2) {
    return 0.0f;
  }

  int16_t raw = (Wire.read() << 8) | Wire.read();
  return (raw / 131.0f) - gyroZOffset;
}

void calibrateGyro() {
  float sum = 0.0f;

  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU_ADDR, 2, true);

    if (Wire.available() >= 2) {
      int16_t raw = (Wire.read() << 8) | Wire.read();
      sum += raw / 131.0f;
    }

    delay(2);
  }

  gyroZOffset = sum / 1000.0f;
}

void initMPU() {
  Wire.begin();
  Wire.setClock(400000);

  mpuWrite(0x6B, 0x00);
  mpuWrite(0x1B, 0x00);
  mpuWrite(0x1A, 0x03);
  delay(100);

  calibrateGyro();
  yawAngle = 0.0f;
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
  yawAngle += (GYRO_Z_SIGN * readGyroZ()) * dt;
}

void resetEncoders() {
  noInterrupts();
  leftCounts = 0;
  rightCounts = 0;
  interrupts();
}

void resetPID() {
  pidIntegral = 0.0f;
  pidLastError = 0.0f;
}

void setMotorRaw(int pwmPin, int in1, int in2, int speed, bool invert) {
  speed = constrain(speed, -255, 255);

  if (invert) {
    speed = -speed;
  }

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
  setMotorRaw(PWMA, AIN1, AIN2, leftSpeed, INVERT_A);
  setMotorRaw(PWMB, BIN1, BIN2, rightSpeed, INVERT_B);
}

void setDrivebaseState(DrivebaseState nextState) {
  drivebaseState = nextState;
  drivebaseStateStart = millis();

  if (DEBUG_DRIVE) {
    Serial.print("Drivebase state -> ");
    Serial.println(static_cast<int>(drivebaseState));
  }
}

}  // namespace

void initDrivebase() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  analogWriteResolution(8);
  analogWriteFrequency(PWMA, 20000);
  analogWriteFrequency(PWMB, 20000);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);

  stopDrivebase();
  initMPU();
}

void stopDrivebase() {
  drive(0, 0);
}

void startStraightMove(float distanceMM, int baseSpeed, int direction, float targetHeading) {
  resetEncoders();
  resetPID();

  activeTargetCounts = static_cast<long>(distanceMM / DIST_PER_COUNT);
  activeBaseSpeed = baseSpeed;
  activeDirection = direction;
  activeTargetHeading = targetHeading;

  if (DEBUG_DRIVE) {
    Serial.print("Start straight move. counts=");
    Serial.print(activeTargetCounts);
    Serial.print(" heading=");
    Serial.println(activeTargetHeading);
  }

  setDrivebaseState(DB_STRAIGHT);
}

void startPivotToHeading(float targetHeading, TurnDirection direction) {
  resetPID();
  activeTargetHeading = targetHeading;
  activeTurnDirection = direction;
  activeTurnStartYaw = yawAngle;
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

  if (drivebaseState == DB_IDLE) {
    return;
  }

  unsigned long now = millis();

  switch (drivebaseState) {
    case DB_STRAIGHT: {
      noInterrupts();
      long lc = leftCounts;
      long rc = rightCounts;
      interrupts();

      long avgCounts = (abs(lc));
      if (avgCounts >= activeTargetCounts) {
        stopDrivebase();
        setDrivebaseState(DB_SETTLE);
        break;
      }

      float error = yawAngle - activeTargetHeading;
      pidIntegral += error * 0.01f;
      pidIntegral = constrain(pidIntegral, -30.0f, 30.0f);
      float derivative = error - pidLastError;
      pidLastError = error;

      float correction = (KP * error) + (KI * pidIntegral) + (KD * derivative);
      correction = constrain(correction, -50.0f, 50.0f);

      int leftSpeed = constrain(activeDirection * activeBaseSpeed + static_cast<int>(correction), -255, 255);
      int rightSpeed = constrain(activeDirection * activeBaseSpeed - static_cast<int>(correction), -255, 255);

      drive(leftSpeed, rightSpeed);
      break;
    }

    case DB_PIVOT_LEFT: {
      float turned = fabsf(yawAngle - activeTurnStartYaw);
      float remaining = activeTurnMagnitude - turned;

      if (remaining <= TURN_STOP_TOLERANCE_DEG || (now - drivebaseStateStart) >= TURN_TIMEOUT_MS) {
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
      float turned = fabsf(yawAngle - activeTurnStartYaw);
      float remaining = activeTurnMagnitude - turned;

      if (remaining <= TURN_STOP_TOLERANCE_DEG || (now - drivebaseStateStart) >= TURN_TIMEOUT_MS) {
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
      if (now - drivebaseStateStart >= TURN_BRAKE_MS) {
        stopDrivebase();
        setDrivebaseState(DB_SETTLE);
      }
      break;

    case DB_SETTLE:
      stopDrivebase();
      if (now - drivebaseStateStart >= PAUSE_MS) {
        setDrivebaseState(DB_IDLE);
      }
      break;

    case DB_IDLE:
    default:
      break;
  }
}
