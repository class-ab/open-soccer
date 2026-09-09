#pragma once

void updateStrategy();

enum class BallState {
        unknown,
        farSides,
        nearOwnGoal,
        nearFarGoal,
        middle,
        ownPossession,
        theirPossession
    };

struct ballLocation {
    bool ballCurrent; // is the ball location current or not?
    float sinceCurrent; // how long since the ball location was current (in seconds)
    BallState ballState; // whether the ball is moving towards our goal or their goal
};

void getBallState(ballLocation &out);