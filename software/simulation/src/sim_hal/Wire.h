#pragma once

#include "sim_hal/Arduino.h"

// Provide a simple Wire.h compatible header for the simulator
using TwoWire = ::TwoWire;
extern TwoWire Wire2;
