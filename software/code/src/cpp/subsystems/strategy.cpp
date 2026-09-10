#include "include/subsystems/strategy.h"
#include "include/subsystems/localization.h"
#include "include/subsystems/robot_state.h"
#include "cpp\subsystems\localization.cpp"
#include "cpp\subsystems\robot_state.cpp"


ballLocation ballState = {false, 0, BallState::unknown};

// Define field zones in millimeters (0,0 is center)
// Adjust these thresholds based on actual field dimensions
// X DIRECTION IS LONG SIDE OF FIELD (i think)
const float MIDDLE_ZONE_X = 615.0;    // ±mm from center x
const float MIDDLE_ZONE_Y = 500.0;    // ±mm from center y
const float SIDE_ZONE_Y = 800.0;      // ±mm from center y for far sides

void updateStrategy() {
    updateBallState();
    updateRobotState();
}

void updateBallState() {
    // get ball localization data
    FieldBall ball;
    getFieldBall(ball);
    
    // update ball state based on localization data
    if (ball.valid) {
        // Ball is valid - update state and mark as current
        ballState.ballCurrent = true;
        ballState.sinceCurrent = 0.0;
        ballState.ballState = determineBallZone(ball);
    } else {
        // Ball is not valid - mark as not current
        ballState.ballCurrent = false;
        ballState.ballState = BallState::unknown;
        // Calculate elapsed time since last valid reading
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

// Determine ball zone based on field coordinates NEEDS TO BE CONFIRMED IRL TUNING WITH FIELD!!!
BallState determineBallZone(const FieldBall &ball) {
    if (abs(ball.xMm) < MIDDLE_ZONE_X && abs(ball.yMm) < MIDDLE_ZONE_Y) {
        return BallState::middle; // ball is in middle zone 
    } else if (abs(ball.yMm) > SIDE_ZONE_Y && ball.xMm < -MIDDLE_ZONE_X) {
        return BallState::farSidesOwn; // ball is in far sides zone AND near own goal
    } else if (abs(ball.yMm) > SIDE_ZONE_Y) {
        return BallState::farSides; // ball is in far sides zone NOT near own goal
    } else if (ball.xMm < -MIDDLE_ZONE_X) {
        return BallState::nearOwnGoal; // ball is near own goal
    } else if (ball.xMm > MIDDLE_ZONE_X) {
        return BallState::nearFarGoal; // ball is near far goal
    }    
    return BallState::unknown;
}



void getBallState(ballLocation &out) {
    out = ballState;
}


