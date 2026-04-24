ENR259_Robot_Project
===============================

What this version does:
- Uses the time based drivebase logic with Teensy 4.0, TB6612FNG, and MPU-6050 heading hold.
- Uses two scooper servos on pins 4 and 5.
- Uses the top sorting mechanism on pins 20 and 17 with the TCS34725.
- Keeps the final 3 top wall dropoff on the direct gates at the end of the mission.
- Keeps sorting non blocking so drivebase timing does not freeze during a sort.

Important fixes made:
- Sorting is enabled again.
- Sorting positions now match the latest tested standalone Teensy sorting code.
- Gyro calibration offset bug was fixed.
- Conveyor now uses the configured CONVEYOR_SPEED instead of a hard coded value.
- Mission now waits for an active sort cycle to finish before lifting the scooper or pivoting.
- Conveyor ownership is handled by the mission state machine instead of setup.

Pins currently used:
- PWMA: 14
- AIN1: 16
- AIN2: 15
- PWMB: 8
- BIN1: 6
- BIN2: 7
- STBY: 9
- Scooper left servo: 4
- Scooper right servo: 5
- Red final gate servo: 11
- White final gate servo: 12
- Blue final gate servo: 13
- Sorting servo: 20
- Sorting gate servo: 17
- Conveyor PWM: 22
- I2C SDA: 18
- I2C SCL: 19

Notes:
- MPU-6050 and TCS34725 share the same I2C bus on pins 18 and 19.
- Encoders remain disabled in this version because the current scooper pin usage conflicts with the old encoder layout.
- If a servo moves the wrong way, swap that servo's open and closed values in Config.h.
- Keep the sensor area empty during startup so ambient calibration is clean.
