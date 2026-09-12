#pragma once

#include "localization.h"

void updateStrategy();
void updateBallState();
void processOpponents();
void updateOpponentState();
void updateRobotState();
void updateRemoteState();

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

enum class RobotState {
    attacking,
    defending,
    damaged,
    bully
};

enum class RobotGoal {
    getBallPush,
    getBallDribble,
    interceptBall,
    pushForward,
    hideForward,
    spinKick,
    kick,
    pass,
    stayBorders
};

struct ballLocation {
    bool ballCurrent; // is the ball location current or not?
    float sinceCurrent; // how long since the ball location was current (in seconds)
    BallState ballState; // whether the ball is moving towards our goal or their goal
};

struct robotLocation {
    RobotState robotState;
    RobotGoal robotGoal;
};

void getBallState(ballLocation &out);