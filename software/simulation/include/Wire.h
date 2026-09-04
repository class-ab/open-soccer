#pragma once
// Simulator Wire compatibility shim. robot.cpp and subsystem files include
// <Wire.h> with angle brackets; this forwards to the simulator's stub in
// sim_hal/Wire.h.
#include "sim_hal/Wire.h"
