#include "include/subsystems/strategy.h"
#include "include/subsystems/localization.h"
#include "include/subsystems/robot_state.h"
#include "cpp\subsystems\localization.cpp"
#include "cpp\subsystems\robot_state.cpp"


ballLocation ballState = {true, 0, BallState::unknown};

void updateStrategy() {
    updateBallState();
    updateRobotState();
}

void updateBallState() {
    // get ball localization data
    FieldBall ball;
    getFieldBall(ball);
    // update ball state based on localization data
    if (!ball.valid) {
        if (ball.xMm < -100) {
            
        }
    } else {
        ballState.ballCurrent = false;
        ballState.ballState = BallState::unknown;
        ballState.sinceCurrent = lastBallPacketMs;
    }
}

void updateRobotState() {
    // get localization data
    RobotPose robot;
    getRobotPose(robot);
    // update robot state based on localization data
    // TODO
}

void getBallState(ballLocation &out) {
    out = ballState;
}


