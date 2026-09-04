#pragma once
// Simulator Arduino compatibility shim. Arduino.h is included with angle
// brackets by robot code (e.g. robot.cpp, attack.cpp, defend.cpp). This
// forwards to the simulator's implementation in sim_hal/Arduino.h.
#include "sim_hal/Arduino.h"
