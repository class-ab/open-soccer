#pragma once

/*
  Attack / striker behaviour (see https://bozotics.github.io/open/strat/attack/).

  The robot has only a single frontal dribbler and no kicker. The behaviour is
  a two-phase state machine driven by the shared `hasBall` flag:

    Phase 1 - Ball chasing (hasBall == false)
      Approach the ball and orbit it so it ends up captured by the frontal
      dribbler. This uses the article's front-catchment offset: the robot
      bearing is R = B + O*M where B is the ball bearing, O is an offset that
      grows with |B| (so the robot goes *around* the ball) and M is a
      multiplier that falls off with ball distance (so it reaches far balls
      efficiently). The robot rotates to face the ball so the front dribbler
      can collect it.

    Phase 2 - Goal scoring (hasBall == true)
      Drive into the opponent goal with the ball. We use the article's front
      scoring orbit (R = G + O_G*M_G, constant multiplier M_G) to approach the
      goal, rotating to face it. Once close enough we stop orbiting and drive
      straight into the goal with the ball (the no-kicker equivalent of a shot).

  Everything is gated by the shared `isAttacking` and `robotCurrentlyRunning`
  flags: when either is false the robot stops and does nothing.
*/

void initAttack();
void attackTick();
