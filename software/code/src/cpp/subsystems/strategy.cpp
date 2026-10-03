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
SIM_TLS bool remoteStateKnown = false;
SIM_TLS bool restoringAsDefender = false;

// Field frame in millimeters: (0,0) is the centre, +X points toward the opponent goal.
const float MIDDLE_ZONE_X = 615.0f;
const float MIDDLE_ZONE_Y = 500.0f;
const float SIDE_ZONE_Y = 800.0f;
const float BORDER_X = 965.0f;
const float BORDER_Y = 660.0f;
const float GOAL_Y = 425.0f;

// Opponent tracking
SIM_TLS OpponentRobot opponent1 = {false, 0.0, 0.0, 0.0, 0};
SIM_TLS OpponentRobot opponent2 = {false, 0.0, 0.0, 0.0, 0};
SIM_TLS int count = 0; // opponent counter

SIM_TLS FieldBall ball;
SIM_TLS RobotPose robotPose;
SIM_TLS FieldBall remoteBall;
SIM_TLS RobotPose remotePose;
SIM_TLS OpponentState opponent1State;
SIM_TLS OpponentState opponent2State;

namespace {
constexpr float OPPONENT_GOAL_X_MM = 989.0f;
constexpr float OPPONENT_GOAL_Y_MM = 0.0f;
constexpr float OWN_GOAL_X_MM = -989.0f;
constexpr float OWN_GOAL_Y_MM = 0.0f;
constexpr float ROBOT_RADIUS_MM = 110.0f;
// The simulator holds the ball about 12.3 cm from the robot centre.
constexpr float BALL_DRIBBLE_CAPTURE_DISTANCE_CM = 10.5f;

// Defence box (own goal area).
constexpr float DEFENCE_BOX_MIN_X_MM = -965.0f;
constexpr float DEFENCE_BOX_MAX_X_MM = -615.0f;
constexpr float DEFENCE_BOX_MAX_ABS_Y_MM = 450.0f;
// backOff starts when the ball is this close to the box, and ends this much further out.
constexpr float BACKOFF_MARGIN_MM = 100.0f;
constexpr float BACKOFF_EXIT_MARGIN_MM = 50.0f;
constexpr float BACKOFF_SPEED = 0.4f;

// Attacker limits (normalized speed, normalized speed per second).
constexpr float ATTACK_ROTATION_SPEED = 0.5f;
constexpr float ATTACK_ROTATION_ACCEL = 4.0f;
constexpr float CENTRE_SPEED = 0.4f;
constexpr float CENTRE_ACCEL = 3.0f;

// behindBall: orbit the ball at ORBIT_RADIUS_MM until the robot is behind it, in line with the goal.
constexpr float ORBIT_RADIUS_MM = 220.0f;
constexpr float ORBIT_SPEED = 0.33f;
// Must exceed the centripetal rate (speed^2 * ROBOT_LINEAR_SPEED_MM_S / radius) or the orbit spirals out.
constexpr float ORBIT_ACCEL = 8.0f;
constexpr float ORBIT_RADIAL_GAIN = 0.01f; // per mm of radius error
constexpr float ORBIT_RADIUS_BLEND_MM = 250.0f;
constexpr float ORBIT_FULL_LAP_DEG = 300.0f;
// An orbit direction is chosen by border clearance only if it beats the other by this much.
constexpr float ORBIT_BORDER_PREFERENCE_MM = 100.0f;

// push: robot-to-line lateral offset (mm) to start / keep pushing.
constexpr float PUSH_START_LATERAL_MM = 80.0f;
constexpr float PUSH_START_DISTANCE_MM = ORBIT_RADIUS_MM + 300.0f;
constexpr float PUSH_MIN_DISTANCE_MM = ROBOT_RADIUS_MM + 50.0f;
constexpr float PUSH_KEEP_LATERAL_MM = 110.0f;
constexpr float PUSH_KEEP_DISTANCE_MM = 600.0f;
constexpr float PUSH_SPEED = 0.43f;
constexpr float PUSH_ACCEL = 12.0f;

// Borders. Clearance is how far the robot centre can still travel before its edge reaches the border.
constexpr float BORDER_ENTER_CLEARANCE_MM = 20.0f;
constexpr float BORDER_EXIT_CLEARANCE_MM = 70.0f;
// Targets are kept at least this far inside the border so they never trigger awayBorders.
constexpr float BORDER_TARGET_MARGIN_MM = 40.0f;
// Slow down over a wider approach zone while keeping the command above the motor deadband.
constexpr float BORDER_SLOW_SPEED = MOTOR_MIN_COMMAND;
constexpr float BORDER_SLOW_RANGE_MM = 500.0f;
constexpr float AWAY_SPEED = 1.0f;
constexpr float AWAY_ACCEL = 40.0f;

// Defender limits.
constexpr float DEFENDER_SPEED = 0.5f;
constexpr float DEFENDER_ROTATION_SPEED = 0.22f;

// A velocity command is issued as a moveTo toward a point this far ahead, so moveTo's
// distance-based slowdown never limits speed.
constexpr float CARROT_MM = 600.0f;

float headingTo(float targetXmm, float targetYmm) {
    return atan2f(targetYmm - robotPose.yMm,
                  targetXmm - robotPose.xMm) * 180.0f / PI;
}

float headingToOpponentGoal() {
    return headingTo(OPPONENT_GOAL_X_MM, OPPONENT_GOAL_Y_MM);
}

float distanceToCentre() {
    return sqrtf(robotPose.xMm * robotPose.xMm + robotPose.yMm * robotPose.yMm);
}

// Beside the goal mouth the end wall is a border; inside the mouth the robot may enter the goal.
float borderClearance(float xMm, float yMm) {
    float clearance = BORDER_Y - ROBOT_RADIUS_MM - fabsf(yMm);
    if (fabsf(yMm) > GOAL_Y - ROBOT_RADIUS_MM) {
        clearance = fminf(clearance, BORDER_X - ROBOT_RADIUS_MM - fabsf(xMm));
    }
    return clearance;
}

float borderSpeedLimit() {
    const float clearance =
        borderClearance(robotPose.xMm, robotPose.yMm) - BORDER_ENTER_CLEARANCE_MM;
    return constrain(BORDER_SLOW_SPEED + (1.0f - BORDER_SLOW_SPEED) *
                     clearance / BORDER_SLOW_RANGE_MM, BORDER_SLOW_SPEED, 1.0f);
}

void clampToField(float &xMm, float &yMm) {
    const float limitY = BORDER_Y - ROBOT_RADIUS_MM - BORDER_TARGET_MARGIN_MM;
    const float limitX = BORDER_X - ROBOT_RADIUS_MM - BORDER_TARGET_MARGIN_MM;
    yMm = constrain(yMm, -limitY, limitY);
    if (fabsf(yMm) > GOAL_Y - ROBOT_RADIUS_MM) {
        xMm = constrain(xMm, -limitX, limitX);
    }
}

bool isBallInBackOffZone(float frontExpansionMm, float negativeYExpansionMm) {
    return ball.valid &&
           ball.xMm >= DEFENCE_BOX_MIN_X_MM &&
           ball.xMm <= DEFENCE_BOX_MAX_X_MM + frontExpansionMm &&
           ball.yMm >= -DEFENCE_BOX_MAX_ABS_Y_MM - negativeYExpansionMm &&
           ball.yMm <= DEFENCE_BOX_MAX_ABS_Y_MM;
}

bool isInOpponentGoalBox(const OpponentRobot &opponent) {
    return opponent.valid && opponent.xMm >= MIDDLE_ZONE_X &&
           opponent.xMm <= BORDER_X && fabsf(opponent.yMm) <= GOAL_Y;
}

// Geometry of the robot relative to the line from the ball to the goal centre.
struct BallLine {
    float unitX;       // ball -> goal
    float unitY;
    float distanceMm;  // robot to ball
    float behindMm;    // robot distance behind the ball along the line (negative: past it)
    float lateralMm;   // robot distance off the line
};

BallLine ballLine() {
    const float goalX = OPPONENT_GOAL_X_MM - ball.xMm;
    const float goalY = OPPONENT_GOAL_Y_MM - ball.yMm;
    const float goalDistance = fmaxf(sqrtf(goalX * goalX + goalY * goalY), 1.0f);
    const float relX = robotPose.xMm - ball.xMm;
    const float relY = robotPose.yMm - ball.yMm;
    BallLine line;
    line.unitX = goalX / goalDistance;
    line.unitY = goalY / goalDistance;
    line.distanceMm = sqrtf(relX * relX + relY * relY);
    line.behindMm = -(relX * line.unitX + relY * line.unitY);
    line.lateralMm = fabsf(line.unitX * relY - line.unitY * relX);
    return line;
}

// Closest approach to a border over a sampled arc of the orbit circle.
float orbitArcClearance(float startDeg, float sweepDeg) {
    float worst = 1.0e6f;
    for (int i = 1; i <= 3; i++) {
        const float rad = (startDeg + sweepDeg * 0.25f * i) * PI / 180.0f;
        const float x = ball.xMm + ORBIT_RADIUS_MM * cosf(rad);
        const float y = ball.yMm + ORBIT_RADIUS_MM * sinf(rad);
        worst = fminf(worst, fminf(BORDER_X - fabsf(x), BORDER_Y - fabsf(y)));
    }
    return worst;
}

// +1 / -1 = counter-clockwise / clockwise, latched until the goal changes.
SIM_TLS float orbitDirection = 0.0f;

// Unit velocity direction that circles the ball toward the point directly behind it.
void getOrbitDirection(float &dirX, float &dirY) {
    const float relX = robotPose.xMm - ball.xMm;
    const float relY = robotPose.yMm - ball.yMm;
    const float radius = fmaxf(sqrtf(relX * relX + relY * relY), 1.0f);
    const float radialX = relX / radius;
    const float radialY = relY / radius;

    const float robotBearing = atan2f(relY, relX) * 180.0f / PI;
    const float behindBearing = atan2f(ball.yMm - OPPONENT_GOAL_Y_MM,
                                       ball.xMm - OPPONENT_GOAL_X_MM) * 180.0f / PI;
    const float error = angleError(behindBearing, robotBearing);
    const float sweepCcw = error >= 0.0f ? error : error + 360.0f;

    if (orbitDirection == 0.0f) {
        const float clearanceCcw = orbitArcClearance(robotBearing, sweepCcw);
        const float clearanceCw = orbitArcClearance(robotBearing, sweepCcw - 360.0f);
        if (fabsf(clearanceCcw - clearanceCw) > ORBIT_BORDER_PREFERENCE_MM) {
            orbitDirection = clearanceCcw > clearanceCw ? 1.0f : -1.0f;
        } else {
            orbitDirection = sweepCcw <= 180.0f ? 1.0f : -1.0f;
        }
    }
    // Overshot the latched direction: go back the short way instead of a full lap.
    const float sweep = orbitDirection > 0.0f ? sweepCcw : 360.0f - sweepCcw;
    if (sweep > ORBIT_FULL_LAP_DEG) {
        orbitDirection = -orbitDirection;
    }

    // Tangent keeps the robot circling; the radial term holds it on the circle.
    const float radiusError = ORBIT_RADIUS_MM - radius;
    const float tangentWeight =
        constrain(1.0f - fabsf(radiusError) / ORBIT_RADIUS_BLEND_MM, 0.35f, 1.0f);
    const float radialWeight = constrain(radiusError * ORBIT_RADIAL_GAIN, -1.5f, 1.5f);
    float vx = orbitDirection * -radialY * tangentWeight + radialX * radialWeight;
    float vy = orbitDirection * radialX * tangentWeight + radialY * radialWeight;
    const float magnitude = fmaxf(sqrtf(vx * vx + vy * vy), 0.0001f);
    dirX = vx / magnitude;
    dirY = vy / magnitude;
}

void moveAlong(float dirX, float dirY, float speed, float accel, float headingDeg) {
    moveTo(robotPose.xMm + dirX * CARROT_MM, robotPose.yMm + dirY * CARROT_MM,
           headingDeg, speed, accel, ATTACK_ROTATION_SPEED, ATTACK_ROTATION_ACCEL);
}

// Defender target: where the ball-to-own-goal line leaves the defence box, else the nearest box point.
void getBallDefenceTarget(float &targetXmm, float &targetYmm) {
    const float deltaX = ball.xMm - OWN_GOAL_X_MM;
    const float deltaY = ball.yMm - OWN_GOAL_Y_MM;
    float tMin = 0.0f;
    float tMax = 1.0f;
    const float origin[2] = {OWN_GOAL_X_MM, OWN_GOAL_Y_MM};
    const float delta[2] = {deltaX, deltaY};
    const float lower[2] = {DEFENCE_BOX_MIN_X_MM, -DEFENCE_BOX_MAX_ABS_Y_MM};
    const float upper[2] = {DEFENCE_BOX_MAX_X_MM, DEFENCE_BOX_MAX_ABS_Y_MM};
    bool intersects = true;
    for (int axis = 0; axis < 2 && intersects; axis++) {
        if (fabsf(delta[axis]) < 0.0001f) {
            intersects = origin[axis] >= lower[axis] && origin[axis] <= upper[axis];
            continue;
        }
        float t1 = (lower[axis] - origin[axis]) / delta[axis];
        float t2 = (upper[axis] - origin[axis]) / delta[axis];
        if (t1 > t2) {
            const float temporary = t1;
            t1 = t2;
            t2 = temporary;
        }
        tMin = fmaxf(tMin, t1);
        tMax = fminf(tMax, t2);
        intersects = tMin <= tMax;
    }

    if (intersects) {
        targetXmm = OWN_GOAL_X_MM + tMax * deltaX;
        targetYmm = OWN_GOAL_Y_MM + tMax * deltaY;
    } else {
        targetXmm = constrain(ball.xMm, DEFENCE_BOX_MIN_X_MM, DEFENCE_BOX_MAX_X_MM);
        targetYmm = constrain(ball.yMm, -DEFENCE_BOX_MAX_ABS_Y_MM, DEFENCE_BOX_MAX_ABS_Y_MM);
    }
    clampToField(targetXmm, targetYmm);
}

void updateDribblerPolicy() {
    const bool attackingWithValidPose = robotPose.valid &&
        localState.robotState == RobotState::attacking;
    if (attackingWithValidPose && localState.robotGoal == RobotGoal::push) {
        setDribblerDirectionReverse();
        setDribblerThrottle(DRIBBLER_RUN_THROTTLE_US);
    } else if (attackingWithValidPose &&
               localState.robotGoal == RobotGoal::behindBall) {
        setDribblerDirectionForward();
        setDribblerThrottle(DRIBBLER_RUN_THROTTLE_US);
    } else {
        stopDribbler();
    }
}
}

void updateStrategy() {
    getFieldBall(ball);
    getRobotPose(robotPose);
    getRemoteFieldBall(remoteBall);
    getRemoteRobotPose(remotePose);
    remoteStateKnown = getRemoteRobotState(remoteState);

    processOpponents();
    updateBallState();
    updateOpponentState();
    updateRobotState();
    updateRobotGoal();
}

void move() {
    SIM_STATIC_TLS RobotGoal previousGoal = RobotGoal::none;
    if (localState.robotGoal != previousGoal) {
        orbitDirection = 0.0f;
        previousGoal = localState.robotGoal;
    }

    if (!robotPose.valid) {
        stopAllDriveMotors();
        stopDribbler();
        return;
    }

    const bool attacking = localState.robotState == RobotState::attacking;
    const float goalHeading = headingToOpponentGoal();

    switch (localState.robotGoal) {
        case RobotGoal::none:
            stopAllDriveMotors();
            break;

        case RobotGoal::awayBorders: {
            const float distance = fmaxf(distanceToCentre(), 1.0f);
            const float heading = attacking ? goalHeading
                : (ball.valid ? headingTo(ball.xMm, ball.yMm) : robotPose.headingDeg);
            moveAlong(-robotPose.xMm / distance, -robotPose.yMm / distance,
                      AWAY_SPEED, AWAY_ACCEL, heading);
            break;
        }

        case RobotGoal::backOff:
        case RobotGoal::searchBall:
            moveTo(0.0f, 0.0f, goalHeading, fminf(BACKOFF_SPEED, borderSpeedLimit()),
                   CENTRE_ACCEL, ATTACK_ROTATION_SPEED, ATTACK_ROTATION_ACCEL);
            break;

        case RobotGoal::behindBall: {
            if (!ball.valid) {
                stopAllDriveMotors();
                break;
            }
            float dirX;
            float dirY;
            getOrbitDirection(dirX, dirY);
            moveAlong(dirX, dirY, fminf(ORBIT_SPEED, borderSpeedLimit()),
                      ORBIT_ACCEL, goalHeading);
            break;
        }

        case RobotGoal::push: {
            if (!ball.valid) {
                stopAllDriveMotors();
                break;
            }
            const float aimX = OPPONENT_GOAL_X_MM - robotPose.xMm;
            const float aimY = OPPONENT_GOAL_Y_MM - robotPose.yMm;
            const float aimDistance = fmaxf(sqrtf(aimX * aimX + aimY * aimY), 1.0f);
            moveAlong(aimX / aimDistance, aimY / aimDistance,
                      fminf(PUSH_SPEED, borderSpeedLimit()), PUSH_ACCEL, goalHeading);
            break;
        }

        case RobotGoal::defendBall: {
            if (!ball.valid) {
                stopAllDriveMotors();
                break;
            }
            float targetXmm;
            float targetYmm;
            getBallDefenceTarget(targetXmm, targetYmm);
            moveTo(targetXmm, targetYmm, headingTo(ball.xMm, ball.yMm),
                   fminf(DEFENDER_SPEED, borderSpeedLimit()), ACCEL_LIMIT,
                   DEFENDER_ROTATION_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }
    }
    updateDribblerPolicy();
}

void processOpponents() {
    OpponentRobot opponentsLocal[2] = {};
    OpponentRobot opponentsRemote[2] = {};
    count = 0;

    getRemoteOpponents(opponentsRemote, 2, count); // COUNT NOT USED FROM REMOTE
    getOpponents(opponentsLocal, 2, count); // count from LOCAl opponents used instead

    OpponentRobot *tracked[2] = {&opponent1, &opponent2};
    for (int i = 0; i < 2; i++) {
        const OpponentRobot &local = opponentsLocal[i];
        const OpponentRobot &remote = opponentsRemote[i];
        OpponentRobot &out = *tracked[i];
        if (count <= i || (!local.valid && !remote.valid)) {
            out.valid = false;
            continue;
        }
        out.valid = true;
        out.timestampMs = 0;
        if (local.valid && remote.valid) {
            out.confidence = (local.confidence + remote.confidence) / 2;
            out.xMm = (local.xMm + remote.xMm) / 2;
            out.yMm = (local.yMm + remote.yMm) / 2;
        } else {
            const OpponentRobot &source = local.valid ? local : remote;
            out.confidence = source.confidence;
            out.xMm = source.xMm;
            out.yMm = source.yMm;
        }
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
		9. unknown (error in validity and location? shouldn't happen!)
		*/
        ballState.ballCurrent = true;
        ballState.sinceCurrent = 0.0;
        if (ball.distanceCm <= BALL_DRIBBLE_CAPTURE_DISTANCE_CM &&
            ball.angleDeg < BALL_TARGET_ANGLE && ball.angleDeg > -BALL_TARGET_ANGLE) {
            ballState.ballPossession = BallPossession::mePossession;
        } else if (remoteBall.valid &&
                   remoteBall.distanceCm <= BALL_DRIBBLE_CAPTURE_DISTANCE_CM &&
                   remoteBall.angleDeg < BALL_TARGET_ANGLE &&
                     remoteBall.angleDeg > -BALL_TARGET_ANGLE) {
                 ballState.ballPossession = BallPossession::himPossession;
		} else if ((opponent1.valid && fabs(ball.xMm - opponent1.xMm) <= opponentBallDistance && fabs(ball.yMm - opponent1.yMm) <= opponentBallDistance)) {
            ballState.ballPossession = BallPossession::theirPossession1; // in possession of opponent 1
        } else if ((opponent2.valid && fabs(ball.xMm - opponent2.xMm) <= opponentBallDistance && fabs(ball.yMm - opponent2.yMm) <= opponentBallDistance)) {
            ballState.ballPossession = BallPossession::theirPossession2; // in possession of opponent 2
        } else if (ball.distanceCm <= BALL_TARGET_DISTANCE_CM &&
                   ball.angleDeg < BALL_TARGET_ANGLE &&
                   ball.angleDeg > -BALL_TARGET_ANGLE) {
            // Ball is close and centered at the dribbler, but not yet confirmed held.
            ballState.ballPossession = BallPossession::front;
		} else {
            ballState.ballPossession = BallPossession::none;
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
            ballState.ballState = BallState::unknown; // edge spot or invalid location
        }
    } else {
		if (ballState.ballPossession == BallPossession::mePossession) {
			ballState.ballPossession = BallPossession::none;
		}
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
    const OpponentRobot *opponents[2] = {&opponent1, &opponent2};
    OpponentState *states[2] = {&opponent1State, &opponent2State};
    for (int i = 0; i < 2; i++) {
        const OpponentRobot &opponent = *opponents[i];
        OpponentState &state = *states[i];
        if (!opponent.valid) {
            state = OpponentState::damaged;
        } else if (isInOpponentGoalBox(opponent)) {
            state = OpponentState::goalie;
        } else if (opponent.xMm >= -MIDDLE_ZONE_X) {
            state = OpponentState::shooting;
        } else if (state != OpponentState::hidingBall) {
            state = OpponentState::chasingBall;
        }
    }
}

void updateRobotState() {
    if (localState.robotState == RobotState::damaged) {
        return;
    }

    if (restoringAsDefender) {
        localState.robotState = RobotState::defending;
        if (remoteStateKnown && remoteState.robotState != RobotState::damaged) {
            restoringAsDefender = false;
        }
    } else if (remoteStateKnown &&
               remoteState.robotState == RobotState::damaged) {
        localState.robotState = RobotState::attacking;
    }
}

void updateRobotGoal() {
    if (localState.robotState == RobotState::damaged || !robotPose.valid) {
        localState.robotGoal = RobotGoal::none;
        return;
    }

    const RobotGoal currentGoal = localState.robotGoal;

    // Borders take priority over everything, and hold until the robot is clear of them.
    const float borderThreshold = currentGoal == RobotGoal::awayBorders
        ? BORDER_EXIT_CLEARANCE_MM : BORDER_ENTER_CLEARANCE_MM;
    if (borderClearance(robotPose.xMm, robotPose.yMm) < borderThreshold) {
        localState.robotGoal = RobotGoal::awayBorders;
        return;
    }

    if (localState.robotState == RobotState::defending) {
        localState.robotGoal = RobotGoal::defendBall;
        return;
    }

    if (!ball.valid) {
        localState.robotGoal = RobotGoal::searchBall;
        return;
    }

    const bool wasBackOff = currentGoal == RobotGoal::backOff;
    const float backOffExpansion = BACKOFF_MARGIN_MM +
        (wasBackOff ? BACKOFF_EXIT_MARGIN_MM : 0.0f);
    if (isBallInBackOffZone(backOffExpansion, backOffExpansion)) {
        localState.robotGoal = RobotGoal::backOff;
        return;
    }

    const BallLine line = ballLine();
    const bool wasPushing = currentGoal == RobotGoal::push;
    const bool pushing = line.behindMm > 0.0f &&
        line.distanceMm >= PUSH_MIN_DISTANCE_MM && (wasPushing
            ? line.lateralMm <= PUSH_KEEP_LATERAL_MM && line.distanceMm <= PUSH_KEEP_DISTANCE_MM
            : line.lateralMm <= PUSH_START_LATERAL_MM && line.distanceMm <= PUSH_START_DISTANCE_MM);
    localState.robotGoal = pushing ? RobotGoal::push : RobotGoal::behindBall;
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
    } else {
        return;
    }
    restoringAsDefender = false;
    selectCurrentRobotNumber(robotNumber);
    localState.robotGoal = RobotGoal::none;
}

void markLocalRobotDamaged() {
    restoringAsDefender = false;
    localState.robotState = RobotState::damaged;
    localState.robotGoal = RobotGoal::none;
}

void restoreLocalRobotAsDefender() {
    if (localState.robotState == RobotState::damaged) {
        restoringAsDefender = true;
        localState.robotState = RobotState::defending;
        localState.robotGoal = RobotGoal::none;
    }
}
