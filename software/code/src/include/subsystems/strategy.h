#pragma once

#include "localization.h"

void updateStrategy();
void updateBallState();
void processOpponents();
void updateOpponentState();
void updateRobotState();
void updateRobotGoal();
void updateLocalRobotMode();
void selectLocalRobotRole(uint8_t robotNumber);
void markLocalRobotDamaged();
void move();

// Opponent tracking
extern OpponentRobot opponent1;
extern OpponentRobot opponent2;

enum class OpponentState {
    damaged, // ALSO "UNKNOWN" !!!
    goalie,
    hidingBall,
    chasingBall,
    shooting
};

enum class BallState {
    unknown,
    farSidesOwn,
    farSides,
    nearOwnGoal,
    nearFarGoal,
    middle
};

enum class BallPossession {
    none,
    mePossession,
    himPossession,
    theirPossession1,
    theirPossession2
};

enum class RobotState {
    attacking,
    defending,
    damaged
};

enum class RobotGoal {
    none,
    getBallPush,
    getBallDribble,
    getBallDribbleAway,
    interceptBall1,
    interceptBall2,
    pushForward,
    hideForward,
    spinKick,
    kick,
    pass,
    awayBorders,
    backOff,
    defendBall,
    defendOpponent1,
    defendOpponent2,
    searchBall
};

struct ballLocation {
    bool ballCurrent; // is the ball location current or not?
    float sinceCurrent; // how long since the ball location was current (in seconds)
    BallState ballState; // whether the ball is moving towards our goal or their goal
    BallPossession ballPossession;
};

struct LocalState {
    RobotState robotState;
    RobotGoal robotGoal;
};

void getBallState(ballLocation &out);
void getLocalState(LocalState &out);
