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

SIM_TLS ballLocation ballState = {false, 0, BallState::unknown};
SIM_TLS LocalState localState = {RobotState::attacking, RobotGoal::none};
SIM_TLS LocalState remoteState = {RobotState::damaged, RobotGoal::none};

// Define field zones in millimeters (0,0 is center)
// Adjust these thresholds based on actual field dimensions
// Field frame: +X points toward the opponent goal; +Y is perpendicular.
const float MIDDLE_ZONE_X = 615.0;    // ±mm from center x
const float MIDDLE_ZONE_Y = 500.0;    // ±mm from center y
const float SIDE_ZONE_Y = 800.0;      // ±mm from center y for far sides
const float BORDER_X = 965.0;
const float BORDER_Y = 660.0;
const float GOAL_Y = 425.0;

// Opponent tracking
SIM_TLS OpponentRobot opponent1 = {false, 0.0, 0.0, 0.0, 0};
SIM_TLS OpponentRobot opponent2 = {false, 0.0, 0.0, 0.0, 0};
SIM_TLS int count = 0; // opponent counter

// LOCAL
SIM_TLS FieldBall ball; // get ball localisation data
SIM_TLS RobotPose robotPose; // get robot localisation data 
// REMOTE
SIM_TLS FieldBall remoteBall;
SIM_TLS RobotPose remotePose;
// OPPONENTS
SIM_TLS OpponentState opponent1State;
SIM_TLS OpponentState opponent2State;

namespace {
constexpr float OPPONENT_GOAL_X_MM = 989.0f;
constexpr float OPPONENT_GOAL_Y_MM = 0.0f;
constexpr float OWN_GOAL_X_MM = -989.0f;
constexpr float OWN_GOAL_Y_MM = 0.0f;
constexpr float BALL_APPROACH_OFFSET_MM = 120.0f;
constexpr float SPIN_KICK_HEADING_TOLERANCE_DEG = 15.0f;
constexpr float BORDER_ESCAPE_STEP_MM = 250.0f;
constexpr float SEARCH_SPIN_RADIUS_MM = 30.0f;
constexpr float DEFENCE_BOX_MIN_X_MM = -965.0f;
constexpr float DEFENCE_BOX_MAX_X_MM = -615.0f;
constexpr float DEFENCE_BOX_MAX_ABS_Y_MM = 450.0f;
constexpr float OWN_GOAL_BACK_X_MM = -989.0f;
// Keep the chassis aligned with the field's +X axis during normal play.
constexpr float PARALLEL_HEADING_DEG = 0.0f;
constexpr float SIDE_WALL_TARGET_Y_MM = 300.0f;
const float DEFENCE_X_MM = -MIDDLE_ZONE_X;

float headingTo(float targetXmm, float targetYmm) {
    return atan2f(targetYmm - robotPose.yMm,
                  targetXmm - robotPose.xMm) * 180.0f / PI;
}

float clampDefenceX(float xMm) {
    return constrain(xMm, DEFENCE_BOX_MIN_X_MM, DEFENCE_BOX_MAX_X_MM);
}

float clampDefenceY(float yMm) {
    return constrain(yMm, -DEFENCE_BOX_MAX_ABS_Y_MM,
                     DEFENCE_BOX_MAX_ABS_Y_MM);
}

bool clipDefenceAxis(float origin, float delta, float minimum, float maximum,
                     float &tMinimum, float &tMaximum) {
    if (fabsf(delta) < 0.0001f) {
        return origin >= minimum && origin <= maximum;
    }

    float t1 = (minimum - origin) / delta;
    float t2 = (maximum - origin) / delta;
    if (t1 > t2) {
        const float temporary = t1;
        t1 = t2;
        t2 = temporary;
    }
    tMinimum = fmaxf(tMinimum, t1);
    tMaximum = fminf(tMaximum, t2);
    return tMinimum <= tMaximum;
}

void getBallDefenceTarget(float &targetXmm, float &targetYmm) {
    const float deltaX = ball.xMm - OWN_GOAL_BACK_X_MM;
    const float deltaY = ball.yMm - OWN_GOAL_Y_MM;
    const float distance = sqrtf(deltaX * deltaX + deltaY * deltaY);
    float tMinimum = 0.0f;
    float tMaximum = 1.0f;
    const bool intersectsBox =
        clipDefenceAxis(OWN_GOAL_BACK_X_MM, deltaX, DEFENCE_BOX_MIN_X_MM,
                        DEFENCE_BOX_MAX_X_MM, tMinimum, tMaximum) &&
        clipDefenceAxis(OWN_GOAL_Y_MM, deltaY,
                        -DEFENCE_BOX_MAX_ABS_Y_MM, DEFENCE_BOX_MAX_ABS_Y_MM,
                        tMinimum, tMaximum);

    if (intersectsBox) {
        const float setbackT = distance > 0.0f
            ? BALL_APPROACH_OFFSET_MM / distance : 0.0f;
        const float targetT = constrain(1.0f - setbackT, tMinimum, tMaximum);
        targetXmm = OWN_GOAL_BACK_X_MM + targetT * deltaX;
        targetYmm = OWN_GOAL_Y_MM + targetT * deltaY;
        return;
    }

    // If the shot line misses the box, hold the nearest legal box point to the ball.
    targetXmm = clampDefenceX(ball.xMm);
    targetYmm = clampDefenceY(ball.yMm);
}

float headingToOpponentGoal() {
    return headingTo(OPPONENT_GOAL_X_MM, OPPONENT_GOAL_Y_MM);
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
    SIM_STATIC_TLS RobotGoal previousGoal = RobotGoal::none;
    SIM_STATIC_TLS bool kickIssued = false;
    SIM_STATIC_TLS bool spinKickTracking = false;
    SIM_STATIC_TLS float spinKickLastHeading = 0.0f;
    SIM_STATIC_TLS float spinKickAccumulatedDeg = 0.0f;
    if (localState.robotGoal != previousGoal) {
        kickIssued = false;
        spinKickTracking = false;
        spinKickAccumulatedDeg = 0.0f;
        previousGoal = localState.robotGoal;
    }

    if (!robotPose.valid) {
        stopMotionAndDribbler();
        return;
    }

    // A border escape always takes priority over every other goal.
    if (localState.robotGoal == RobotGoal::awayBorders) {
        const float distanceToCenter = sqrtf(robotPose.xMm * robotPose.xMm +
                                             robotPose.yMm * robotPose.yMm);
        const float step = fminf(distanceToCenter, BORDER_ESCAPE_STEP_MM);
        const float scale = distanceToCenter > 0.0f ? step / distanceToCenter : 0.0f;
        const float targetX = robotPose.xMm * (1.0f - scale);
        const float targetY = robotPose.yMm * (1.0f - scale);
        const float targetHeading = ball.valid
            ? headingTo(ball.xMm, ball.yMm) : robotPose.headingDeg;
        stopDribbler();
        moveTo(targetX, targetY, targetHeading,
               0.4f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
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
                   PARALLEL_HEADING_DEG, 0.8f, ACCEL_LIMIT,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;

        case RobotGoal::getBallDribble:
            if (!ball.valid) {
                stopMotionAndDribbler();
                break;
            }
            dribbleForward();
            moveTo(ball.xMm, ball.yMm, headingTo(ball.xMm, ball.yMm),
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
                   headingTo(ball.xMm, ball.yMm), 0.8f, ACCEL_LIMIT,
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
            const float targetHeading = ball.valid
                ? headingTo(ball.xMm, ball.yMm)
                : headingTo(opponent.xMm, opponent.yMm);
            moveTo(opponent.xMm, opponent.yMm, targetHeading, 1.0f, ACCEL_LIMIT,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }

        case RobotGoal::pushForward:
            dribbleForward();
            moveTo(OPPONENT_GOAL_X_MM, OPPONENT_GOAL_Y_MM,
                   PARALLEL_HEADING_DEG, 1.0f, ACCEL_LIMIT,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;

        case RobotGoal::hideForward: {
            // Travel toward the own-goal-side corner while facing 45 degrees off -X.
            const float side = robotPose.yMm >= 0.0f ? 1.0f : -1.0f;
            dribbleForward();
            moveTo(OWN_GOAL_X_MM, side * SIDE_WALL_TARGET_Y_MM,
                   180.0f - side * 45.0f,
                   0.8f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }

        case RobotGoal::spinKick: {
            const float goalHeading = headingToOpponentGoal();
            stopDribbler();
            if (!spinKickTracking) {
                spinKickLastHeading = robotPose.headingDeg;
                spinKickAccumulatedDeg = 0.0f;
                spinKickTracking = true;
            }
            spinKickAccumulatedDeg +=
                fabsf(angleError(robotPose.headingDeg, spinKickLastHeading));
            spinKickLastHeading = robotPose.headingDeg;
            if (!kickIssued && spinKickAccumulatedDeg >= 360.0f &&
                fabsf(angleError(goalHeading, robotPose.headingDeg)) <=
                    SPIN_KICK_HEADING_TOLERANCE_DEG) {
                kick();
                kickIssued = true;
            }
            stopAllDriveMotors();
            if (!kickIssued) {
                drive(0.0f, 0.0f, ROTATION_MAX_SPEED);
            }
            break;
        }

        case RobotGoal::kick: {
            const float goalHeading = headingToOpponentGoal();
            stopDribbler();
            moveTo(robotPose.xMm, robotPose.yMm, goalHeading,
                   0.0f, 0.0f, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            if (!kickIssued && fabsf(angleError(goalHeading, robotPose.headingDeg)) <=
                                   SPIN_KICK_HEADING_TOLERANCE_DEG) {
                kick();
                kickIssued = true;
            }
            break;
        }

        case RobotGoal::pass: {
            if (!remotePose.valid) {
                stopMotionAndDribbler();
                break;
            }
            stopDribbler();
            const float passHeading = headingTo(remotePose.xMm, remotePose.yMm);
            moveTo(robotPose.xMm, robotPose.yMm,
                   passHeading, 0.0f, 0.0f,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            if (!kickIssued && fabsf(angleError(passHeading,
                                                robotPose.headingDeg)) <=
                                   SPIN_KICK_HEADING_TOLERANCE_DEG) {
                kick();
                kickIssued = true;
            }
            break;
        }

        case RobotGoal::backOff:
            stopDribbler();
            moveTo(DEFENCE_X_MM, OWN_GOAL_Y_MM, PARALLEL_HEADING_DEG,
                   0.7f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;

        case RobotGoal::defendBall: {
            if (!ball.valid) {
                stopMotionAndDribbler();
                break;
            }
            stopDribbler();
            float targetXmm;
            float targetYmm;
            getBallDefenceTarget(targetXmm, targetYmm);
            moveTo(targetXmm, targetYmm,
                   headingTo(ball.xMm, ball.yMm),
                   1.0f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }

        case RobotGoal::defendOpponent1:
        case RobotGoal::defendOpponent2: {
            const OpponentRobot &opponent =
                localState.robotGoal == RobotGoal::defendOpponent1 ? opponent1 : opponent2;
            if (!opponent.valid) {
                stopMotionAndDribbler();
                break;
            }
            stopDribbler();
            moveTo(clampDefenceX(opponent.xMm), clampDefenceY(opponent.yMm),
                   headingTo(opponent.xMm, opponent.yMm),
                   0.9f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }

        case RobotGoal::searchBall:
            stopDribbler();
            if (sqrtf(robotPose.xMm * robotPose.xMm +
                      robotPose.yMm * robotPose.yMm) > SEARCH_SPIN_RADIUS_MM) {
                moveTo(0.0f, 0.0f, robotPose.headingDeg,
                       0.5f, ACCEL_LIMIT, 0.0f, ROTATION_ACCEL_LIMIT);
            } else {
                stopAllDriveMotors();
                drive(0.0f, 0.0f, 0.15f);
            }
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
            opponent2.valid = true;
            opponent2.confidence = (opponentsLocal[1].confidence + opponentsRemote[1].confidence) / 2;
            opponent2.xMm = (opponentsLocal[1].xMm + opponentsRemote[1].xMm) / 2;
            opponent2.yMm = (opponentsLocal[1].yMm + opponentsRemote[1].yMm) / 2;
            opponent2.timestampMs = 0;
        } else if (opponentsLocal[1].valid == true) {
            opponent2.valid = true;
            opponent2.confidence = opponentsLocal[1].confidence;
            opponent2.xMm = opponentsLocal[1].xMm;
            opponent2.yMm = opponentsLocal[1].yMm;
            opponent2.timestampMs = 0;
        } else {
            opponent2.valid = true;
            opponent2.confidence = opponentsRemote[1].confidence;
            opponent2.xMm = opponentsRemote[1].xMm;
            opponent2.yMm = opponentsRemote[1].yMm;
            opponent2.timestampMs = 0;
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
        } else if (remoteBall.distanceCm <= BALL_TARGET_DISTANCE_CM && remoteBall.angleDeg < BALL_TARGET_ANGLE && remoteBall.angleDeg > -BALL_TARGET_ANGLE) {
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
            if (opponent1.xMm >= MIDDLE_ZONE_X) {
                opponent1State = OpponentState::goalie;
            } else if (opponent1.xMm >= -MIDDLE_ZONE_X) {
                opponent1State = OpponentState::shooting;
            } 
        } else {
            if (opponent1.xMm >= MIDDLE_ZONE_X) {
                opponent1State = OpponentState::goalie;
            } else if (opponent1.xMm >= -MIDDLE_ZONE_X) {
                opponent1State = OpponentState::shooting;
            } else {
                opponent1State = OpponentState::chasingBall;
            }
        }
    }
    if (opponent2.valid == false) {
        opponent2State = OpponentState::damaged;
    } else {
        if (opponent2State == OpponentState::hidingBall) {
            if (opponent2.xMm >= MIDDLE_ZONE_X) {
                opponent2State = OpponentState::goalie;
            } else if (opponent2.xMm >= -MIDDLE_ZONE_X) {
                opponent2State = OpponentState::shooting;
            } 
        } else {
            if (opponent2.xMm >= MIDDLE_ZONE_X) {
                opponent2State = OpponentState::goalie;
            } else if (opponent2.xMm >= -MIDDLE_ZONE_X) {
                opponent2State = OpponentState::shooting;
            } else {
                opponent2State = OpponentState::chasingBall;
            }
        }
    }
}

void updateRobotState() {
    // Damaged robots become defenders while the remote (non damaged) become attackers
    if (localState.robotState == RobotState::damaged) {
        localState.robotState = RobotState::defending;
    } else if (remoteState.robotState == RobotState::damaged) {
        localState.robotState = RobotState::attacking;
        return;
    }

    uint8_t robotNumber = (localState.robotState == RobotState::attacking) ? 1 : 2;
    selectCurrentRobotNumber(robotNumber);
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
