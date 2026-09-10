#include "include/subsystems/strategy.h"
#include "include/subsystems/localization.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/vision.h"
#include "cpp\subsystems\localization.cpp"
#include "cpp\subsystems\robot_state.cpp"
#include "cpp\subsystems\vision.cpp"


ballLocation ballState = {false, 0, BallState::unknown};

// Define field zones in millimeters (0,0 is center)
// Adjust these thresholds based on actual field dimensions
// X DIRECTION IS LONG SIDE OF FIELD (i think)
const float MIDDLE_ZONE_X = 615.0;    // ±mm from center x
const float MIDDLE_ZONE_Y = 500.0;    // ±mm from center y
const float SIDE_ZONE_Y = 800.0;      // ±mm from center y for far sides

void updateStrategy() {
    updateBallState();
}

void updateBallState() {
    FieldBall ball; // get ball localisation data
    getFieldBall(ball);
    RobotPose robot; // get robot localisation data 
    getRobotPose(robot);

    // update ball state based on localization data
    if (ball.valid) {
        // Ball is valid - update state and mark as current
        ballState.ballCurrent = true;
        ballState.sinceCurrent = 0.0;
        if (ball.distanceCm <= BALL_TARGET_DISTANCE_CM && ball.angleDeg < 5 && ball.angleDeg > -5) {
            ballState.ballState = BallState::mePossession;
        } /* else if ( other robot ballState.ballState == "mePossession")
        {
            NEED TO GET COMMUNICATION HERE

        }*/else if (abs(ball.xMm) < MIDDLE_ZONE_X && abs(ball.yMm) < MIDDLE_ZONE_Y) {
            ballState.ballState = BallState::middle; // ball is in middle zone 
        } else if (abs(ball.yMm) > SIDE_ZONE_Y && ball.xMm < -MIDDLE_ZONE_X) {
          ballState.ballState = BallState::farSidesOwn; // ball is in far sides zone AND near own goal
        } else if (abs(ball.yMm) > SIDE_ZONE_Y) {
           ballState.ballState = BallState::farSides; // ball is in far sides zone NOT near own goal
        } else if (ball.xMm < -MIDDLE_ZONE_X) {
          ballState.ballState = BallState::nearOwnGoal; // ball is near own goal
        } else if (ball.xMm > MIDDLE_ZONE_X) {
        ballState.ballState = BallState::nearFarGoal; // ball is near far goal
        } else {
        ballState.ballState = BallState::unknown; // cannot find ball most likely, or it is in an edge spot
        }
    } else {
        // Ball is not valid - mark as not current
        ballState.ballCurrent = false;
        ballState.ballState = BallState::unknown;
        // Calculate elapsed time since last valid reading
        ballState.sinceCurrent = lastBallPacketMs;
    }

}

void getBallState(ballLocation &out) {
    out = ballState;
}