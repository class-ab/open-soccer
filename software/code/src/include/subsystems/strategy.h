#pragma once

void updateStrategy();
void updateBallState();

enum class BallState {
        unknown,
        farSidesOwn,
        farSides,
        nearOwnGoal,
        nearFarGoal,
        middle,
        mePossession,
        himPossession,
        theirPossession
    };

struct ballLocation {
    bool ballCurrent; // is the ball location current or not?
    float sinceCurrent; // how long since the ball location was current (in seconds)
    BallState ballState; // whether the ball is moving towards our goal or their goal
};

void getBallState(ballLocation &out);