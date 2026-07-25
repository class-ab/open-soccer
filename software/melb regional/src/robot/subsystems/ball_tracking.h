#ifndef SUBSYSTEMS_BALL_TRACKING_H
#define SUBSYSTEMS_BALL_TRACKING_H

#include <Arduino.h>

// Sets up the CS-framed, bit-banged SPI-style receiver pins and attaches
// the CS/clock ISRs. Call once from setup() - safe to call before the
// OpenMV cam powers up, it just sits idle until CS/SCK edges start
// arriving. See ball_tracking.cpp for the wiring and packet format.
void initBallTracking();

// Drains whatever the CS/clock ISRs have queued up, validates the
// checksum + sync byte, and - if valid - updates the latest ball data.
// Call every tick; systemTick() does this from loop() and from inside
// move()'s/chaseTick()'s own blocking loops.
void processBallPacket();

// Runs one tick of "slowly drive towards the ball," using the freshest
// packet from the OpenMV cam. Meant to be called repeatedly (e.g. every
// loop() while a button is held) - see the long comment above its
// definition in ball_tracking.cpp for the approach/converge behavior.
void chaseTick();

#endif
