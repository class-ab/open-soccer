#include "include/subsystems/strategy.h"
#include "include/subsystems/localization.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/vision.h"
#include "include/subsystems/communication.h"
#include "include/subsystems/robot_config.h"
#include "string"
#include "iostream"

const int opponentBallDistance = 30;

ballLocation ballState = {false, 0, BallState::unknown};

// Define field zones in millimeters (0,0 is center)
// Adjust these thresholds based on actual field dimensions
// X DIRECTION IS LONG SIDE OF FIELD (i think)
const float MIDDLE_ZONE_X = 615.0;    // ±mm from center x
const float MIDDLE_ZONE_Y = 500.0;    // ±mm from center y
const float SIDE_ZONE_Y = 800.0;      // ±mm from center y for far sides

// Opponent tracking
OpponentRobot opponent1 = {false, 0.0, 0.0, 0.0, 0};
OpponentRobot opponent2 = {false, 0.0, 0.0, 0.0, 0};

void updateStrategy() {
    updateBallState();
}

void updateBallState() {
	// LOCAL 
    FieldBall ball; // get ball localisation data
    getFieldBall(ball);
    RobotPose robotPose; // get robot localisation data 
    getRobotPose(robotPose);
	// REMOTE
	FieldBall remoteBall;
	getRemoteFieldBall(remoteBall);
	RobotPose remotePose;
	getRemoteRobotPose(remotePose);
	// LOCAL OPPONENTS
    processOpponents();

	std::string lastBallState = "ballState.ballState"; // use for identifying if ball is hidden 

    if (ball.valid) {  // update ball state based on localization data
		/* order of detection:
		1. own possesion (in dribbler)
		2. him possession (in remote dribbler)
		3. opponent possession, 1 and 2
		4. middle zone
		5. far sides AND own goal
		6. far sides (!"")
		7. near own goal
		8. near far goal
		9. unknown (error in validity and location? shouldn't happen!)
		*/
        ballState.ballCurrent = true;
        ballState.sinceCurrent = 0.0;
        if (ball.distanceCm <= BALL_TARGET_DISTANCE_CM && ball.angleDeg < 5 && ball.angleDeg > -5) {
            ballState.ballState = BallState::mePossession;
        } else if (remoteBall.distanceCm <= BALL_TARGET_DISTANCE_CM && remoteBall.angleDeg < 5 && remoteBall.angleDeg > -5) {
			ballState.ballState = BallState::himPossession;
		} else if ((opponent1.valid && fabs(ball.xMm - opponent1.xMm) <= opponentBallDistance && fabs(ball.yMm - opponent1.yMm) <= opponentBallDistance)) {
            ballState.ballState = BallState::theirPossession1; // in possession of opponent 1
        } else if ((opponent2.valid && fabs(ball.xMm - opponent2.xMm) <= opponentBallDistance && fabs(ball.yMm - opponent2.yMm) <= opponentBallDistance)) {
			ballState.ballState = BallState::theirPossession2; // in possession of opponent 2
		} else if (abs(ball.xMm) < MIDDLE_ZONE_X && abs(ball.yMm) < MIDDLE_ZONE_Y) {
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
            ballState.ballState = BallState::unknown; // edge spot or some error between validity and location
        }
    } else {
		if (bool lastBallState = "theirPossession1") {
			ballState.ballCurrent = true; // ball IS current, ASSUMED
        	ballState.ballState = BallState::theirPossession1; // maintain possession state
       	 	ballState.sinceCurrent = lastBallPacketMs; // BUT still show that it is ASSUMED, OLD data
		} else if (bool lastBallState = "theirPossession2") {
			ballState.ballCurrent = true; 
        	ballState.ballState = BallState::theirPossession2;
       	 	ballState.sinceCurrent = lastBallPacketMs;
		}
        // Ball is not valid - mark as not current
        ballState.ballCurrent = false;
        ballState.ballState = BallState::unknown;
        // Calculate elapsed time since last valid reading
        ballState.sinceCurrent = lastBallPacketMs;
    }

}

void processOpponents() {
    OpponentRobot opponents[2];
    int count = 0;

    getOpponents(opponents, 2, count);

    // Assign first opponent if available
    if (count >= 1) {
        opponent1 = opponents[0];
    } else {
        opponent1.valid = false;
    }

    // Assign second opponent if available
    if (count >= 2) {
        opponent2 = opponents[1];
    } else {
        opponent2.valid = false;
    }
}

void getBallState(ballLocation &out) {
    out = ballState;
}