#pragma once

#include "localization.h"

void updateStrategy();
void updateBallState();
void processOpponents();

// Opponent tracking
extern OpponentRobot opponent1;
extern OpponentRobot opponent2;

enum class BallState {
        unknown,
        farSidesOwn,
        farSides,
        nearOwnGoal,
        nearFarGoal,
        middle,
        mePossession,
        himPossession,
        theirPossession1,
        theirPossession2
    };

struct ballLocation {
    bool ballCurrent; // is the ball location current or not?
    float sinceCurrent; // how long since the ball location was current (in seconds)
    BallState ballState; // whether the ball is moving towards our goal or their goal
};

void getBallState(ballLocation &out);