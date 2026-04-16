ENR259_Robot_Project
===============================

What this version does:
- Uses the message (4) style drivebase logic with the Teensy 4.0, TB6612FNG, MPU-6050, encoders, and heading hold.
- Uses the message (4) scooper motion in a non blocking state machine.
- Keeps the integrated multi file project structure like your original RobotIntegrated code.
- Adds the original style sorting subsystem back in as a separate module.
- Keeps the final top wall dropoff on the 3 direct gates instead of the sorting mechanism.

Behavior:
1. Run the 5 lane pattern once.
2. During the mission, the sorting subsystem keeps reading the TCS34725 and routing balls with the sorting servo and small sorting gate.
3. After each lane, run the scooper cycle.
4. After lane 5, turn sideways so the robot is along the top wall.
5. Open the red final gate right there.
6. Move forward 1.5 ft along the wall.
7. Open the white final gate.
8. Move forward 1.5 ft again.
9. Open the blue final gate.
10. Stop and stay stopped.

Files:
- ENR259_Robot_Project.ino
- Config.h
- Drivebase.cpp / Drivebase.h
- Sorting.cpp / Sorting.h
- Scooper.cpp / Scooper.h
- Gates.cpp / Gates.h
- Mission.cpp / Mission.h

Pins currently used:
- PWMA: 14
- AIN1: 16
- AIN2: 15
- PWMB: 8
- BIN1: 6
- BIN2: 7
- STBY: 9
- Left encoder A: 2
- Left encoder B: 4
- Right encoder A: 1
- Right encoder B: 3
- Scooper servo: 10
- Red final gate servo: 11
- White final gate servo: 12
- Blue final gate servo: 13
- Sorting servo: 5
- Sorting gate servo: 17
- Conveyor PWM: 22

Important notes:
- The 3 final drop gates use 0 degrees = closed and 90 degrees = open.
- The sorting mechanism uses its own positions from your original integrated code.
- Pins 18 and 19 are avoided because I2C is used for the MPU and color sensor.
- If a servo moves the wrong way, swap that servo's open and closed values in Config.h.
- If the TCS34725 does not initialize, the robot will still drive and do the final 3 gate dropoff, but continuous sorting will stay inactive.


Sorting note:
- Sorting keeps the same color thresholds and sequence from your standalone Storting.ino
  but uses the safer non blocking state machine here so the same ball is not retriggered
  over and over while it is still under the sensor.
