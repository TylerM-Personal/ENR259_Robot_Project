# ENR259_Robot_Project

ENR 259 robot project with code, wiring, CAD, testing, and documentation.

## Drive Log For Drift Troubleshooting

This version includes a RAM-backed drive log for straight-drive runs. While the robot is driving straight, the Teensy stores compact samples in memory instead of printing every loop. After each straight move ends, it prints a CSV block over Serial.

Use this when troubleshooting a robot that drifts right or left while trying to drive straight.

Student collection steps:

1. Upload the current code to the Teensy.
2. Open the Serial Monitor at `115200` baud.
3. Start recording video before starting the robot. Try to capture the whole robot and the floor/field reference lines.
4. Let the robot run one straight-drive segment.
5. After the segment ends, copy everything from `DRIVE_LOG_BEGIN` through `DRIVE_LOG_END`.
6. Send the copied drive log together with the matching video.

The CSV columns are:

```text
ms,moveMs,sampleDtMs,loopDtMs,state,baseSpeed,direction,target,yaw,error,p,i,d,rawCorrection,correction,leftSpeed,rightSpeed,gyroZ,flags
```

Helpful notes:

- `moveMs` is the time since the current straight move started.
- `yaw`, `target`, and `error` are in degrees.
- `p`, `i`, and `d` show the PID terms used to compute correction.
- `leftSpeed` and `rightSpeed` are the final PWM commands sent to the motors.
- `gyroZ` is the corrected gyro Z rate used for yaw integration.
- `flags = 1` means the correction hit the configured clamp.

Logging settings are in `ENR259_Robot_Project/Config.h`:

```cpp
ENABLE_DRIVE_LOG
DRIVE_LOG_AUTO_PRINT
DRIVE_LOG_SAMPLE_MS
DRIVE_LOG_MAX_SAMPLES
```
