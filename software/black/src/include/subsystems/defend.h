#pragma once

/*
  Defend / defensive behaviour for when the robot is NOT attacking.

  The robot returns to a waiting point a fixed distance in front of its own
  goal and constantly points at the ball (so it is ready to react). This uses
  the localisation module to navigate to the field point; only the translation
  comes from localisation -- the orientation is steered here so the robot
  keeps facing the ball rather than the waypoint.
*/

#include "include/subsystems/localisation.h"

void initDefend();
void defendTick();
