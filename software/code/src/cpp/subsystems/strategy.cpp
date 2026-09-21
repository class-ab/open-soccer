#include "include/subsystems/strategy.h"
#include "include/subsystems/localization.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/vision.h"
#include "include/subsystems/communication.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/drivebase.h"
#include "include/subsystems/dribbler.h"
#include "include/subsystems/imu.h"
#include "string"
#include "iostream"

const int opponentBallDistance = 30; // mm

ballLocation ballState = {false, 0, BallState::unknown};
LocalState localState = {RobotState::attacking, RobotGoal::none};
LocalState remoteState = {RobotState::damaged, RobotGoal::none};

// Define field zones in millimeters (0,0 is center)
// Adjust these thresholds based on actual field dimensions
// X DIRECTION IS LONG SIDE OF FIELD (i think)
const float MIDDLE_ZONE_X = 615.0;    // ±mm from center x
const float MIDDLE_ZONE_Y = 500.0;    // ±mm from center y
const float SIDE_ZONE_Y = 800.0;      // ±mm from center y for far sides
const float BORDER_X = 965.0;
const float BORDER_Y = 660.0;
const float GOAL_Y = 425.0;

// Opponent tracking
OpponentRobot opponent1 = {false, 0.0, 0.0, 0.0, 0};
OpponentRobot opponent2 = {false, 0.0, 0.0, 0.0, 0};
int count = 0; // opponent counter

// LOCAL
FieldBall ball; // get ball localisation data
RobotPose robotPose; // get robot localisation data 
// REMOTE
FieldBall remoteBall;
RobotPose remotePose;
// OPPONENTS
OpponentState opponent1State;
OpponentState opponent2State;

namespace {
constexpr float OPPONENT_GOAL_HEADING_DEG = 0.0f;
constexpr float BALL_APPROACH_OFFSET_MM = 120.0f;
constexpr float FORWARD_TRAVEL_MM = 1000.0f;
constexpr float SPIN_KICK_HEADING_TOLERANCE_DEG = 15.0f;
constexpr float SIDE_WALL_TARGET_Y_MM = 800.0f;
const float DEFENCE_X_MM = -MIDDLE_ZONE_X;

float headingTo(float targetXmm, float targetYmm) {
    return atan2f(targetYmm - robotPose.yMm,
                  targetXmm - robotPose.xMm) * 180.0f / PI;
}

float headingToBallOrCurrent() {
    return ball.valid ? headingTo(ball.xMm, ball.yMm) : robotPose.headingDeg;
}

void dribbleForward() {
    setDribblerDirectionForward();
    setDribblerThrottle(DRIBBLER_RUN_THROTTLE_US);
}

void stopMotionAndDribbler() {
    stopAllDriveMotors();
    stopDribbler();
}
}

void updateStrategy() {
    getFieldBall(ball);
    getRobotPose(robotPose);
    getRemoteFieldBall(remoteBall);
    getRemoteRobotPose(remotePose);
    getRemoteRobotState(remoteState);

    processOpponents();
    updateBallState();
    updateOpponentState();
    updateRobotState();
    updateRobotGoal();
}

void move() {
    // Kicking is edge-triggered: repeatedly calling kick() restarts its timer
    // and would prevent the kicker sequence from completing.
    static RobotGoal previousGoal = RobotGoal::none;
    static bool kickIssued = false;
    if (localState.robotGoal != previousGoal) {
        kickIssued = false;
        previousGoal = localState.robotGoal;
    }

    // A border escape always takes priority over every other goal.
    if (localState.robotGoal == RobotGoal::awayBorders) {
        stopDribbler();
        moveTo(0.0f, 0.0f, headingToBallOrCurrent(),
               0.4f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
        return;
    }

    if (!robotPose.valid) {
        stopMotionAndDribbler();
        return;
    }

    switch (localState.robotGoal) {
        case RobotGoal::none:
            stopMotionAndDribbler();
            break;

        case RobotGoal::getBallPush:
            if (!ball.valid) {
                stopMotionAndDribbler();
                break;
            }
            // Approach from the own-goal side, already facing the opponent goal.
            dribbleForward();
            moveTo(ball.xMm - BALL_APPROACH_OFFSET_MM, ball.yMm,
                   OPPONENT_GOAL_HEADING_DEG, 0.8f, ACCEL_LIMIT,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;

        case RobotGoal::getBallDribble:
            if (!ball.valid) {
                stopMotionAndDribbler();
                break;
            }
            dribbleForward();
            moveTo(ball.xMm, ball.yMm, headingToBallOrCurrent(),
                   0.7f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;

        case RobotGoal::getBallDribbleAway:
            if (!ball.valid) {
                stopMotionAndDribbler();
                break;
            }
            // Continue through the ball in the +X (opponent-goal) direction.
            dribbleForward();
            moveTo(ball.xMm + BALL_APPROACH_OFFSET_MM, ball.yMm,
                   headingToBallOrCurrent(), 0.8f, ACCEL_LIMIT,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;

        case RobotGoal::interceptBall1:
        case RobotGoal::interceptBall2: {
            const OpponentRobot &opponent =
                localState.robotGoal == RobotGoal::interceptBall1 ? opponent1 : opponent2;
            if (!opponent.valid) {
                stopMotionAndDribbler();
                break;
            }
            dribbleForward();
            // When the ball is visible, face its side while driving into the opponent.
            const float heading = ball.valid ? headingToBallOrCurrent()
                                             : headingTo(opponent.xMm, opponent.yMm);
            moveTo(opponent.xMm, opponent.yMm, heading, 1.0f, ACCEL_LIMIT,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }

        case RobotGoal::pushForward:
            dribbleForward();
            moveTo(robotPose.xMm + FORWARD_TRAVEL_MM, robotPose.yMm,
                   OPPONENT_GOAL_HEADING_DEG, 1.0f, ACCEL_LIMIT,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;

        case RobotGoal::hideForward: {
            // At either side wall, head diagonally toward that wall and the own goal.
            const float side = robotPose.yMm >= 0.0f ? 1.0f : -1.0f;
            dribbleForward();
            moveTo(robotPose.xMm - FORWARD_TRAVEL_MM, side * SIDE_WALL_TARGET_Y_MM,
                   atan2f(side, -1.0f) * 180.0f / PI,
                   0.8f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }

        case RobotGoal::spinKick:
            stopDribbler();
            moveTo(robotPose.xMm, robotPose.yMm, OPPONENT_GOAL_HEADING_DEG,
                   0.0f, 0.0f, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            if (!kickIssued && fabsf(angleError(OPPONENT_GOAL_HEADING_DEG,
                                                robotPose.headingDeg)) <=
                                   SPIN_KICK_HEADING_TOLERANCE_DEG) {
                kick();
                kickIssued = true;
            }
            break;

        case RobotGoal::kick:
            stopDribbler();
            moveTo(robotPose.xMm, robotPose.yMm, OPPONENT_GOAL_HEADING_DEG,
                   0.0f, 0.0f, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            if (!kickIssued && fabsf(angleError(OPPONENT_GOAL_HEADING_DEG,
                                                robotPose.headingDeg)) <=
                                   SPIN_KICK_HEADING_TOLERANCE_DEG) {
                kick();
                kickIssued = true;
            }
            break;

        case RobotGoal::pass:
            if (!remotePose.valid) {
                stopMotionAndDribbler();
                break;
            }
            stopDribbler();
            moveTo(robotPose.xMm, robotPose.yMm,
                   headingTo(remotePose.xMm, remotePose.yMm), 0.0f, 0.0f,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            if (!kickIssued && fabsf(angleError(headingTo(remotePose.xMm, remotePose.yMm),
                                                robotPose.headingDeg)) <=
                                   SPIN_KICK_HEADING_TOLERANCE_DEG) {
                kick();
                kickIssued = true;
            }
            break;

        case RobotGoal::backOff:
            stopDribbler();
            moveTo(DEFENCE_X_MM, 0.0f, headingToBallOrCurrent(),
                   0.7f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;

        case RobotGoal::defendBall:
            if (!ball.valid) {
                stopMotionAndDribbler();
                break;
            }
            stopDribbler();
            moveTo(ball.xMm, ball.yMm, headingToBallOrCurrent(),
                   0.9f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;

        case RobotGoal::defendOpponent1:
        case RobotGoal::defendOpponent2: {
            const OpponentRobot &opponent =
                localState.robotGoal == RobotGoal::defendOpponent1 ? opponent1 : opponent2;
            if (!opponent.valid) {
                stopMotionAndDribbler();
                break;
            }
            stopDribbler();
            moveTo(opponent.xMm, opponent.yMm, headingTo(opponent.xMm, opponent.yMm),
                   0.9f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }

        case RobotGoal::searchBall:
            stopDribbler();
            // Keep the heading target ahead of the current pose so it continues to spin.
            moveTo(0.0f, 0.0f, robotPose.headingDeg + 45.0f,
                   0.5f, ACCEL_LIMIT, 0.15f, ROTATION_ACCEL_LIMIT);
            break;

        case RobotGoal::awayBorders:
            // Handled above so it cannot be pre-empted by another action.
            break;
    }
}

void processOpponents() {
    OpponentRobot opponentsLocal[2];
    OpponentRobot opponentsRemote[2];
    count = 0;

    getRemoteOpponents(opponentsRemote, 2, count); // COUNT NOT USED FROM REMOTE
    getOpponents(opponentsLocal, 2, count); // count from LOCAl opponents used instead 

    // Assign first opponent if available
    if (count >= 1) {
        if ((opponentsLocal[0].valid == true) && (opponentsRemote[0].valid == true)) {
            opponent1.valid = true;
            opponent1.confidence = (opponentsLocal[0].confidence + opponentsRemote[0].confidence) / 2;
            opponent1.xMm = (opponentsLocal[0].xMm + opponentsRemote[0].xMm) / 2;
            opponent1.yMm = (opponentsLocal[0].yMm + opponentsRemote[0].yMm) / 2;
            opponent1.timestampMs = 0;
        } else if (opponentsLocal[0].valid == true) {
            opponent1.valid = true;
            opponent1.confidence = opponentsLocal[0].confidence;
            opponent1.xMm = opponentsLocal[0].xMm;
            opponent1.yMm = opponentsLocal[0].yMm;
            opponent1.timestampMs = 0;
        } else {
            opponent1.valid = true;
            opponent1.confidence = opponentsRemote[0].confidence;
            opponent1.xMm = opponentsRemote[0].xMm;
            opponent1.yMm = opponentsRemote[0].yMm;
            opponent1.timestampMs = 0;
        }        
    } else {
        opponent1.valid = false;
    }

    // Assign second opponent if available
    if (count >= 2) {
       if ((opponentsLocal[1].valid == true) && (opponentsRemote[1].valid == true)) {
            opponent1.valid = true;
            opponent1.confidence = (opponentsLocal[1].confidence + opponentsRemote[1].confidence) / 2;
            opponent1.xMm = (opponentsLocal[1].xMm + opponentsRemote[1].xMm) / 2;
            opponent1.yMm = (opponentsLocal[1].yMm + opponentsRemote[1].yMm) / 2;
            opponent1.timestampMs = 0;
        } else if (opponentsLocal[1].valid == true) {
            opponent1.valid = true;
            opponent1.confidence = opponentsLocal[1].confidence;
            opponent1.xMm = opponentsLocal[1].xMm;
            opponent1.yMm = opponentsLocal[1].yMm;
            opponent1.timestampMs = 0;
        } else {
            opponent1.valid = true;
            opponent1.confidence = opponentsRemote[1].confidence;
            opponent1.xMm = opponentsRemote[1].xMm;
            opponent1.yMm = opponentsRemote[1].yMm;
            opponent1.timestampMs = 0;
        }   
    } else {
        opponent2.valid = false;
    }
}

void updateBallState() {
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
            ballState.ballPossession = BallPossession::mePossession;
        } else if (remoteBall.distanceCm <= BALL_TARGET_DISTANCE_CM && remoteBall.angleDeg < 5 && remoteBall.angleDeg > -5) {
			ballState.ballPossession = BallPossession::himPossession;
		} else if ((opponent1.valid && fabs(ball.xMm - opponent1.xMm) <= opponentBallDistance && fabs(ball.yMm - opponent1.yMm) <= opponentBallDistance)) {
            ballState.ballPossession = BallPossession::theirPossession1; // in possession of opponent 1
        } else if ((opponent2.valid && fabs(ball.xMm - opponent2.xMm) <= opponentBallDistance && fabs(ball.yMm - opponent2.yMm) <= opponentBallDistance)) {
			ballState.ballPossession = BallPossession::theirPossession2; // in possession of opponent 2
		} 
        if (abs(ball.xMm) < MIDDLE_ZONE_X && abs(ball.yMm) < MIDDLE_ZONE_Y) {
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
		if (lastBallState == "theirPossession1") {
			ballState.ballCurrent = true; // ball IS current, ASSUMED
        	ballState.ballPossession = BallPossession::theirPossession1; // maintain possession state
       	 	ballState.sinceCurrent = lastBallPacketMs; // BUT still show that it is ASSUMED, OLD data
            opponent1State = OpponentState::hidingBall;
		} else if (lastBallState == "theirPossession2") {
			ballState.ballCurrent = true; 
        	ballState.ballPossession = BallPossession::theirPossession2;
       	 	ballState.sinceCurrent = lastBallPacketMs;
            opponent2State = OpponentState::hidingBall;
		}
        // Ball is not valid - mark as not current
        ballState.ballCurrent = false;
        ballState.ballState = BallState::unknown;
        // Calculate elapsed time since last valid reading
        ballState.sinceCurrent = lastBallPacketMs;
    }

}

void updateOpponentState() {
    if (opponent1.valid == false) {
        opponent1State = OpponentState::damaged;
    } else {
        if (opponent1State == OpponentState::hidingBall) { // checks if hiding ball first
            if (opponent1.xMm >= -MIDDLE_ZONE_X) { // CHECK POSITIVE / NEGATIVE X/Y DIRECTIONS !!!!
                opponent1State = OpponentState::shooting; // if hiding ball, then can only be shooting
            } 
        } else {
            if (opponent1.xMm >= -MIDDLE_ZONE_X) { // CHECK POSITIVE / NEGATIVE X/Y DIRECTIONS !!!!
                opponent1State = OpponentState::shooting; // else just check normally 
            } else if (opponent1.xMm >= MIDDLE_ZONE_X) {
                opponent1State = OpponentState::goalie;
            } else {
                opponent1State = OpponentState::chasingBall;
            }
        }
    }
    if (opponent2.valid == false) {
        opponent2State = OpponentState::damaged;
    } else {
        if (opponent2State == OpponentState::hidingBall) {
            if (opponent2.xMm >= -MIDDLE_ZONE_X) { // CHECK POSITIVE / NEGATIVE X/Y DIRECTIONS !!!!
                opponent2State = OpponentState::shooting;
            } 
        } else {
            if (opponent2.xMm >= -MIDDLE_ZONE_X) { // CHECK POSITIVE / NEGATIVE X/Y DIRECTIONS !!!!
                opponent2State = OpponentState::shooting;
            } else if (opponent2.xMm >= MIDDLE_ZONE_X) {
                opponent2State = OpponentState::goalie;
            } else {
                opponent2State = OpponentState::chasingBall;
            }
        }
    }
}

void updateRobotState() {
    // Selected roles are fixed: robot 1 attacks and robot 2 defends. Damage
    // remains latched until a role-selection button is pressed again.
    if (localState.robotState == RobotState::damaged) {
        return;
    }

    localState.robotState = (getCurrentRobotNumber() == 1)
                                ? RobotState::attacking
                                : RobotState::defending;
}

void updateRobotGoal() {
    if (localState.robotState == RobotState::damaged) {
        localState.robotGoal = RobotGoal::none; // DAMAGED
    }

    if (abs(robotPose.yMm) >= BORDER_Y || (abs(robotPose.yMm) >= GOAL_Y && abs(robotPose.xMm) >= BORDER_X)) {
        localState.robotGoal = RobotGoal::awayBorders; // FIRST CHECK BORDERS
    } else if (localState.robotState == RobotState::attacking) { // IF ATTACKING:
        if (ballState.ballPossession == BallPossession::mePossession) { // IF I HAVE THE BALL:
            if (ballState.ballState == BallState::middle) {
                localState.robotGoal = RobotGoal::pushForward;
            } else if (ballState.ballState == BallState::farSides 
                       || ballState.ballState == BallState::farSidesOwn 
                       || ballState.ballState == BallState::nearOwnGoal) {
                localState.robotGoal = RobotGoal::hideForward;
            } else if (ballState.ballState == BallState::nearFarGoal) {
                localState.robotGoal = RobotGoal::kick;
            } else {
                if (ball.valid == true) {
                    localState.robotGoal = RobotGoal::pushForward;
                } else {
                    localState.robotGoal = RobotGoal::none;
                }
            }
        } else if (ballState.ballPossession == BallPossession::himPossession) {
            localState.robotGoal = RobotGoal::defendBall;
        } else if (ballState.ballPossession == BallPossession::none) {
            if (ballState.ballState == BallState::nearOwnGoal) {
                localState.robotGoal = RobotGoal::backOff;
            } else if (ballState.ballState == BallState::middle) {
                localState.robotGoal = RobotGoal::getBallPush;
            } else if (ballState.ballState == BallState::farSides) {
                localState.robotGoal = RobotGoal::getBallDribble;
            } else if (ballState.ballState == BallState::nearFarGoal) {
                localState.robotGoal = RobotGoal::getBallPush;
            } else if (ballState.ballState == BallState::farSidesOwn) {
                localState.robotGoal = RobotGoal::getBallDribbleAway;
            } else {
                localState.robotGoal = RobotGoal::searchBall;
            }
        } else if (ballState.ballPossession == BallPossession::theirPossession1) {
            localState.robotGoal = RobotGoal::interceptBall1;
        } else if (ballState.ballPossession == BallPossession::theirPossession2) {
            localState.robotGoal = RobotGoal::interceptBall2;
        }
    } else if (localState.robotState == RobotState::defending) {
        if (opponent1.valid == true && opponent1State == OpponentState::shooting) {
            localState.robotGoal = RobotGoal::defendOpponent1;
        } else if (opponent2.valid == true  && opponent2State == OpponentState::shooting) {
            localState.robotGoal = RobotGoal::defendOpponent2;
        } else if (ballState.ballState == BallState::farSidesOwn || ballState.ballState == BallState::nearOwnGoal) {
            localState.robotGoal = RobotGoal::getBallDribbleAway;
        } else {
            localState.robotGoal = RobotGoal::defendBall;
        }
    }
}

void updateLocalRobotMode() {
    switch (localState.robotState) {
        case RobotState::attacking:
            localState.robotState = RobotState::defending;
            break;
        case RobotState::defending:
            localState.robotState = RobotState::damaged;
            break;
        case RobotState::damaged:
            localState.robotState = RobotState::attacking;
            break;
    }
}

void getBallState(ballLocation &out) {
    out = ballState;
}

void getLocalState(LocalState &out) {
    out = localState;
}

void selectLocalRobotRole(uint8_t robotNumber) {
    if (robotNumber == 1) {
        localState.robotState = RobotState::attacking;
    } else if (robotNumber == 2) {
        localState.robotState = RobotState::defending;
    }
    localState.robotGoal = RobotGoal::none;
}

void markLocalRobotDamaged() {
    localState.robotState = RobotState::damaged;
    localState.robotGoal = RobotGoal::none;
}
