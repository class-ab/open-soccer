#ifndef SUBSYSTEMS_IMU_H
#define SUBSYSTEMS_IMU_H

#include <Arduino.h>

// Current yaw heading, degrees, updated by updateIMU(). Read by the
// drivebase (field-oriented drive(), heading-hold PID) and by
// ball_tracking's chaseTick() to convert the ball's chassis-relative
// bearing into a world-frame heading target.
extern float currentYawDeg;

// Starts the BNO08x over I2C, enables the rotation-vector report, and
// takes an initial heading reading. If the sensor isn't found, prints an
// error and hangs forever - matches the original startup behavior, since
// the robot isn't safe to drive without a working IMU. Call once from
// setup().
void initIMU();

// Pulls the newest sensor event (if any) and updates currentYawDeg.
// Also re-enables the rotation-vector report if the sensor reports it
// was reset. Non-blocking - safe to call every loop()/systemTick() tick.
void updateIMU();

#endif
