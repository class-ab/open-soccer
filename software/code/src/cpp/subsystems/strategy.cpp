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

// Define field zones in millimeters (0,0 is center)
// Adjust these thresholds based on actual field dimensions
// Field frame: +X points toward the opponent goal; +Y is perpendicular.
const float MIDDLE_ZONE_X = 615.0f;    // ±mm from center x
const float MIDDLE_ZONE_Y = 500.0f;    // ±mm from center y
const float SIDE_ZONE_Y = 800.0f;      // ±mm from center y for far sides
const float BORDER_X = 965.0f;
const float BORDER_Y = 660.0f;
const float GOAL_Y = 425.0f;

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
constexpr float GOAL_PUSH_DEPTH_MM = 100.0f;
constexpr float ROBOT_DIAMETER_MM = 220.0f;
constexpr float ROBOT_RADIUS_MM = ROBOT_DIAMETER_MM * 0.5f;
constexpr float KICK_LINE_X_MM = 615.0f;
constexpr float KICK_LINE_TOLERANCE_MM = 40.0f;
constexpr float OWN_GOAL_X_MM = -989.0f;
constexpr float OWN_GOAL_Y_MM = 0.0f;
constexpr float BALL_APPROACH_OFFSET_MM = 120.0f;
// The simulator holds the ball about 12.3 cm from the robot centre.
constexpr float BALL_DRIBBLE_CAPTURE_DISTANCE_CM = 10.5f;
constexpr float GOAL_SHOT_OFFSET_Y_MM = 250.0f;
// Ram starts when the ball is this close and within this angle of the robot-goal line.
constexpr float RAM_START_DISTANCE_CM = 20.0f;
constexpr float RAM_START_LINEUP_DEG = 3.0f;
constexpr float RAM_KEEP_DISTANCE_CM = 45.0f;
constexpr float RAM_KEEP_LINEUP_DEG = 30.0f;
// Orbit radius around the ball; must stay inside RAM_START_DISTANCE_CM.
constexpr float ORBIT_RADIUS_MM = 160.0f;
constexpr float ORBIT_MAX_STEP_DEG = 45.0f;
constexpr float ORBIT_SPEED = 0.70f;
constexpr float ORBIT_ACCEL_LIMIT = 4.0f;
// Multiplies the heading error during the ram: at full speed the normal heading gain is too weak
// to stop the robot yawing and curving the ball off the goal line.
constexpr float RAM_HEADING_GAIN = 4.0f;
constexpr float RAM_HEADING_ERROR_MAX_DEG = 90.0f;
// How far past the ball the ram target is placed, so the robot drives through it.
constexpr float BALL_RAM_EXTEND_MM = 400.0f;
// The dribbler spins in reverse (bouncing the ball away) only when the ball is this far upfield.
constexpr float DRIBBLER_KICK_ZONE_X_MM = 400.0f;
constexpr float SPIN_KICK_HEADING_TOLERANCE_DEG = 15.0f;
constexpr float BORDER_ESCAPE_STEP_MM = 400.0f;
constexpr float BORDER_ESCAPE_SPEED = 0.90f;
constexpr float BORDER_ESCAPE_ACCEL_LIMIT = 6.0f;
// Escape triggers this far before the border, and holds until this much extra clearance.
constexpr float BORDER_EARLY_MARGIN_MM = 90.0f;
constexpr float BORDER_EXIT_MARGIN_MM = 60.0f;
// Also trigger on where the robot will be this long from now at its commanded velocity.
constexpr float BORDER_LOOKAHEAD_S = 0.15f;
constexpr float ATTACKER_BORDER_ESCAPE_STEP_MM = 350.0f;
constexpr float ATTACKER_BORDER_ESCAPE_SPEED = 0.75f;
constexpr float ATTACKER_BORDER_ESCAPE_ACCEL_LIMIT = 4.5f;
constexpr float ATTACKER_BORDER_EARLY_MARGIN_MM = 65.0f;
constexpr float ATTACKER_BORDER_EXIT_MARGIN_MM = 45.0f;
constexpr float ATTACKER_BORDER_LOOKAHEAD_S = 0.10f;
constexpr float SEARCH_SPIN_RADIUS_MM = 75.0f;
constexpr float SEARCH_RESUME_RADIUS_MM = 125.0f;
constexpr float DEFENCE_BOX_MIN_X_MM = -965.0f;
constexpr float DEFENCE_BOX_MAX_X_MM = -615.0f;
constexpr float DEFENCE_BOX_MAX_ABS_Y_MM = 450.0f;
constexpr float DRIBBLER_HALF_FIELD_HYSTERESIS_MM = 75.0f;
constexpr float OWN_GOAL_BACK_X_MM = -989.0f;
constexpr float SIDE_WALL_TARGET_Y_MM = 300.0f;
constexpr float HIDE_BALL_TARGET_X_MM = 580.0f;
constexpr float HIDE_BALL_LANE_Y_MM = 350.0f;

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
    float tMinimum = 0.0f;
    float tMaximum = 1.0f;
    const bool intersectsBox =
        clipDefenceAxis(OWN_GOAL_BACK_X_MM, deltaX, DEFENCE_BOX_MIN_X_MM,
                        DEFENCE_BOX_MAX_X_MM, tMinimum, tMaximum) &&
        clipDefenceAxis(OWN_GOAL_Y_MM, deltaY,
                        -DEFENCE_BOX_MAX_ABS_Y_MM, DEFENCE_BOX_MAX_ABS_Y_MM,
                        tMinimum, tMaximum);

    if (intersectsBox) {
        const float targetT = tMaximum;
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

/*
bool isAtKickLine() {
    return robotPose.xMm >= KICK_LINE_X_MM - KICK_LINE_TOLERANCE_MM;
}
*/

bool isBallInGoalPushZone() {
    return ball.valid &&
           ball.xMm >= OPPONENT_GOAL_X_MM - GOAL_PUSH_DEPTH_MM &&
           ball.xMm <= OPPONENT_GOAL_X_MM &&
           fabsf(ball.yMm) <= GOAL_Y;
}

bool isInOpponentGoalBox(const OpponentRobot &opponent) {
    return opponent.valid && opponent.xMm >= MIDDLE_ZONE_X &&
           opponent.xMm <= BORDER_X && fabsf(opponent.yMm) <= GOAL_Y;
}

bool isBallInDefenceBox() {
    return ball.valid && ball.xMm >= DEFENCE_BOX_MIN_X_MM &&
           ball.xMm <= DEFENCE_BOX_MAX_X_MM &&
           fabsf(ball.yMm) <= DEFENCE_BOX_MAX_ABS_Y_MM;
}

/*
bool isSideRoute() {
    return (ball.valid && fabsf(ball.yMm) >= MIDDLE_ZONE_Y) ||
           ballState.ballState == BallState::farSides ||
           ballState.ballState == BallState::farSidesOwn;
}

float hideBallSide() {
    if (ball.valid) {
        return ball.yMm < 0.0f ? -1.0f : 1.0f;
    }
    return robotPose.yMm < 0.0f ? -1.0f : 1.0f;
}

bool hideBallReached() {
    const float side = hideBallSide();
    return robotPose.xMm >= HIDE_BALL_TARGET_X_MM - POSITION_TOLERANCE_MM &&
           fabsf(robotPose.yMm - side * HIDE_BALL_LANE_Y_MM) <= 45.0f;
}

bool isBehindBallRoute(RobotGoal goal) {
    return goal == RobotGoal::getBallPush ||
           goal == RobotGoal::pushForward ||
           goal == RobotGoal::scoring;
}
*/

float shotTargetY() {
    const OpponentRobot *goalie = nullptr;
    if (isInOpponentGoalBox(opponent1)) {
        goalie = &opponent1;
    }
    if (isInOpponentGoalBox(opponent2) &&
        (!goalie || opponent2.xMm > goalie->xMm)) {
        goalie = &opponent2;
    }

    if (!goalie) {
        return OPPONENT_GOAL_Y_MM;
    }

    float targetYmm = goalie->yMm >= 0.0f
        ? -GOAL_SHOT_OFFSET_Y_MM : GOAL_SHOT_OFFSET_Y_MM;
    if (fabsf(goalie->yMm) < 30.0f) {
        targetYmm = robotPose.yMm >= 0.0f
            ? -GOAL_SHOT_OFFSET_Y_MM : GOAL_SHOT_OFFSET_Y_MM;
    }
    return targetYmm;
}

// +1 / -1 = counter-clockwise / clockwise orbit, latched until the goal changes.
SIM_TLS float orbitDirection = 0.0f;

float ballBearingPointDistanceToCentre(float bearingDeg) {
    const float rad = bearingDeg * PI / 180.0f;
    const float x = ball.xMm + ORBIT_RADIUS_MM * cosf(rad);
    const float y = ball.yMm + ORBIT_RADIUS_MM * sinf(rad);
    return sqrtf(x * x + y * y);
}

// Next point on a circle around the ball, stepped from the robot's bearing toward the
// side of the ball directly opposite the goal, going around the field-centre side.
void getBallOrbitTarget(float &targetXmm, float &targetYmm) {
    const float robotBearing = atan2f(robotPose.yMm - ball.yMm,
                                      robotPose.xMm - ball.xMm) * 180.0f / PI;
    const float behindBearing = atan2f(ball.yMm - OPPONENT_GOAL_Y_MM,
                                       ball.xMm - OPPONENT_GOAL_X_MM) * 180.0f / PI;
    const float error = angleError(behindBearing, robotBearing);
    const float sweepCcw = error >= 0.0f ? error : error + 360.0f;
    if (orbitDirection == 0.0f) {
        const float midCcw = robotBearing + sweepCcw * 0.5f;
        const float midCw = robotBearing - (360.0f - sweepCcw) * 0.5f;
        orbitDirection = ballBearingPointDistanceToCentre(midCcw) <=
                         ballBearingPointDistanceToCentre(midCw) ? 1.0f : -1.0f;
    }
    float sweep = orbitDirection > 0.0f ? sweepCcw : 360.0f - sweepCcw;
    // Overshot the latched direction: finish the short way instead of a full lap.
    if (sweep > 300.0f) {
        orbitDirection = -orbitDirection;
        sweep = 360.0f - sweep;
    }
    const float step = orbitDirection * fminf(sweep, ORBIT_MAX_STEP_DEG);
    const float bearingRad = (robotBearing + step) * PI / 180.0f;
    targetXmm = ball.xMm + ORBIT_RADIUS_MM * cosf(bearingRad);
    targetYmm = ball.yMm + ORBIT_RADIUS_MM * sinf(bearingRad);
}

// True when the robot, ball and goal centre are in line, with the ball in between.
bool isLinedUpBehindBall(float toleranceDeg) {
    return fabsf(angleError(headingToOpponentGoal(),
                            headingTo(ball.xMm, ball.yMm))) <= toleranceDeg;
}

/*
void dribbleForward() {
    setDribblerDirectionForward();
    setDribblerThrottle(DRIBBLER_RUN_THROTTLE_US);
}
*/

float ballApproachSpeed() {
    const float distanceScale = ball.valid
        ? constrain(ball.distanceCm / 80.0f, 0.30f, 1.0f)
        : 1.0f;
    const float requestedSpeed = BALL_APPROACH_MAX_SPEED * distanceScale;
    const float movementFloor = fminf(MOTOR_MIN_COMMAND,
                                      BALL_APPROACH_MAX_SPEED);
    return fmaxf(requestedSpeed, movementFloor);
}

/* Dribbling policy disabled: the robot no longer tries to capture or carry the ball.
SIM_TLS bool defenderDribblerActive = false;

void updateDribblerPolicy() {
    bool shouldRun = false;
    if (localState.robotState != RobotState::defending) {
        defenderDribblerActive = false;
    }
    if (robotPose.valid && localState.robotState == RobotState::defending) {
        const bool defendingOpponent =
            localState.robotGoal == RobotGoal::defendOpponent1 ||
            localState.robotGoal == RobotGoal::defendOpponent2;
        if (defendingOpponent) {
            defenderDribblerActive = false;
        } else if (ball.valid) {
            if (ball.xMm < -DRIBBLER_HALF_FIELD_HYSTERESIS_MM) {
                defenderDribblerActive = true;
            } else if (ball.xMm > DRIBBLER_HALF_FIELD_HYSTERESIS_MM) {
                defenderDribblerActive = false;
            }
        }
        shouldRun = defenderDribblerActive && !defendingOpponent;
    } else if (robotPose.valid && localState.robotState == RobotState::attacking) {
        const bool hasBall = ballState.ballPossession == BallPossession::mePossession ||
                             ballState.ballPossession == BallPossession::front;
        switch (localState.robotGoal) {
            case RobotGoal::getBallPush:
            case RobotGoal::getBallDribble:
            case RobotGoal::getBallDribbleAway:
            case RobotGoal::interceptBall1:
            case RobotGoal::interceptBall2:
            case RobotGoal::pushForward:
            case RobotGoal::hideForward:
            case RobotGoal::spinKick:
            case RobotGoal::kick:
            case RobotGoal::pass:
            case RobotGoal::awayBorders:
            case RobotGoal::defendBall:
            case RobotGoal::defendOpponent1:
            case RobotGoal::defendOpponent2:
            case RobotGoal::searchBall:
            case RobotGoal::dribbleForward:
            case RobotGoal::hideBall:
            case RobotGoal::scoring:
                shouldRun = true;
                break;
            case RobotGoal::none:
            case RobotGoal::backOff:
                shouldRun = hasBall;
                break;
        }
    }

    if (shouldRun) {
        dribbleForward();
    } else {
        stopDribbler();
    }
}
*/

// The dribbler only acts as a kicker: spun against its capture direction near the
// opponent goal, it bounces the ball away toward the goal.
void updateDribblerPolicy() {
    const bool ramGoal = localState.robotGoal == RobotGoal::getBallPush ||
                        localState.robotGoal == RobotGoal::kick ||
                        localState.robotGoal == RobotGoal::scoring;
    if (robotPose.valid && localState.robotState == RobotState::attacking &&
        ramGoal && ball.valid && ball.xMm >= DRIBBLER_KICK_ZONE_X_MM) {
        setDribblerDirectionReverse();
        setDribblerThrottle(DRIBBLER_RUN_THROTTLE_US);
    } else {
        stopDribbler();
    }
}

bool isNearBorder(float xMm, float yMm, float marginMm) {
    return fabsf(yMm) >= BORDER_Y - ROBOT_RADIUS_MM - marginMm ||
           (fabsf(yMm) >= GOAL_Y - ROBOT_RADIUS_MM - marginMm &&
            fabsf(xMm) >= BORDER_X - ROBOT_RADIUS_MM - marginMm);
}

void stopMotion() {
    stopAllDriveMotors();
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
    SIM_STATIC_TLS bool searchSettled = false;
    // Latched once aligned with the goal: translation toward the goal is enabled.
    SIM_STATIC_TLS bool goalDriveStarted = false;
    SIM_STATIC_TLS bool spinKickTracking = false;
    SIM_STATIC_TLS float spinKickLastHeading = 0.0f;
    SIM_STATIC_TLS float spinKickAccumulatedDeg = 0.0f;
    if (localState.robotGoal != previousGoal) {
        goalDriveStarted = false;
        orbitDirection = 0.0f;
        spinKickTracking = false;
        spinKickAccumulatedDeg = 0.0f;
        searchSettled = false;
        previousGoal = localState.robotGoal;
    }

    if (!robotPose.valid) {
        stopMotion();
        stopDribbler();
        return;
    }

    // A border escape always takes priority over every other goal.
    if (localState.robotGoal == RobotGoal::awayBorders) {
        const bool attacking = localState.robotState == RobotState::attacking;
        const float distanceToCenter = sqrtf(robotPose.xMm * robotPose.xMm +
                                             robotPose.yMm * robotPose.yMm);
        const float step = fminf(distanceToCenter,
            attacking ? ATTACKER_BORDER_ESCAPE_STEP_MM : BORDER_ESCAPE_STEP_MM);
        const float scale = distanceToCenter > 0.0f ? step / distanceToCenter : 0.0f;
        const float targetX = robotPose.xMm * (1.0f - scale);
        const float targetY = robotPose.yMm * (1.0f - scale);
        const float targetHeading =
            attacking
                ? headingToOpponentGoal()
                : (ball.valid ? headingTo(ball.xMm, ball.yMm)
                              : robotPose.headingDeg);
        moveTo(targetX, targetY, targetHeading,
               attacking ? ATTACKER_BORDER_ESCAPE_SPEED : BORDER_ESCAPE_SPEED,
               attacking ? ATTACKER_BORDER_ESCAPE_ACCEL_LIMIT : BORDER_ESCAPE_ACCEL_LIMIT,
               ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
        updateDribblerPolicy();
        return;
    }

    switch (localState.robotGoal) {
        case RobotGoal::none:
            stopMotion();
            break;

        case RobotGoal::getBallPush: {
            if (!ball.valid) {
                stopMotion();
                break;
            }
            // Face the goal and orbit the ball; kick (ram) takes over once lined up.
            float targetXmm;
            float targetYmm;
            getBallOrbitTarget(targetXmm, targetYmm);
            moveTo(targetXmm, targetYmm, headingToOpponentGoal(), ORBIT_SPEED,
                   ORBIT_ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }

        // Dribbling goals are no longer selected; original code kept below.
        case RobotGoal::getBallDribble:
        case RobotGoal::dribbleForward:
        case RobotGoal::getBallDribbleAway:
        case RobotGoal::pushForward:
        case RobotGoal::hideBall:
            stopMotion();
            break;

        /*
        case RobotGoal::getBallDribble:
            if (!ball.valid) {
                stopAllDriveMotors();
                break;
            }
            moveTo(ball.xMm, ball.yMm, headingTo(ball.xMm, ball.yMm),
                     ballApproachSpeed(), BALL_APPROACH_ACCEL_LIMIT,
                     ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;

        case RobotGoal::dribbleForward:
            if (!ball.valid) {
                stopAllDriveMotors();
                break;
            }
            if (ballState.ballPossession == BallPossession::mePossession ||
                ballState.ballPossession == BallPossession::front) {
                moveTo(KICK_LINE_X_MM, robotPose.yMm,
                      headingToOpponentGoal(), POSSESSION_MAX_SPEED, ACCEL_LIMIT,
                       ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            } else {
                moveTo(ball.xMm, ball.yMm, headingTo(ball.xMm, ball.yMm),
                       ballApproachSpeed(), BALL_APPROACH_ACCEL_LIMIT,
                       ROTATION_MAX_SPEED,
                       ROTATION_ACCEL_LIMIT);
            }
            break;

        case RobotGoal::getBallDribbleAway:
            if (!ball.valid) {
                stopMotion();
                break;
            }
            // Continue through the ball in the +X (opponent-goal) direction.
            moveTo(ball.xMm + BALL_APPROACH_OFFSET_MM, ball.yMm,
                     headingTo(ball.xMm, ball.yMm), ballApproachSpeed(),
                     BALL_APPROACH_ACCEL_LIMIT,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        */

        case RobotGoal::interceptBall1:
        case RobotGoal::interceptBall2: {
            const OpponentRobot &opponent =
                localState.robotGoal == RobotGoal::interceptBall1 ? opponent1 : opponent2;
            if (!opponent.valid) {
                stopMotion();
                break;
            }
            const float targetHeading = headingToOpponentGoal();
                 moveTo(opponent.xMm, opponent.yMm, targetHeading,
                     BALL_APPROACH_MAX_SPEED, BALL_APPROACH_ACCEL_LIMIT,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }

        /*
        case RobotGoal::pushForward:
            if (ballState.ballPossession != BallPossession::front &&
                ballState.ballPossession != BallPossession::mePossession) {
                stopMotion();
                break;
            }
            // Carry the ball to the kick line, then hold position for the shot.
            moveTo(KICK_LINE_X_MM, robotPose.yMm,
                     headingToOpponentGoal(), POSSESSION_MAX_SPEED, ACCEL_LIMIT,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        */

        case RobotGoal::scoring:
            if (!ball.valid ||
                (ballState.ballPossession != BallPossession::front &&
                 ballState.ballPossession != BallPossession::mePossession)) {
                stopMotion();
                break;
            }
            {
                const float goalwardX = OPPONENT_GOAL_X_MM - ball.xMm;
                const float goalwardY = OPPONENT_GOAL_Y_MM - ball.yMm;
                const float goalwardDistance =
                    sqrtf(goalwardX * goalwardX + goalwardY * goalwardY);
                const float goalwardScale = goalwardDistance > 0.001f
                    ? BALL_APPROACH_OFFSET_MM / goalwardDistance : 0.0f;
                const float targetXmm = OPPONENT_GOAL_X_MM +
                                        goalwardX * goalwardScale;
                const float targetYmm = OPPONENT_GOAL_Y_MM +
                                        goalwardY * goalwardScale;
                const float targetHeading = goalwardDistance > 0.001f
                    ? atan2f(goalwardY, goalwardX) * 180.0f / PI
                    : headingToOpponentGoal();
                  moveTo(targetXmm, targetYmm, targetHeading, BALL_RAM_SPEED,
                       BALL_RAM_ACCEL_LIMIT, ROTATION_MAX_SPEED,
                       ROTATION_ACCEL_LIMIT);
            }
            break;

        /*
        case RobotGoal::hideBall: {
            const float side = hideBallSide();
            const float laneY = side * HIDE_BALL_LANE_Y_MM;
            const bool onLane = fabsf(robotPose.yMm - laneY) <= 45.0f;
            moveTo(onLane ? HIDE_BALL_TARGET_X_MM : robotPose.xMm,
                   laneY, side * 90.0f,
                   onLane ? 0.65f : 0.45f, ACCEL_LIMIT,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }
        */

        case RobotGoal::hideForward: {
            // Travel toward the own-goal-side corner while facing 45 degrees off -X.
            const float side = robotPose.yMm >= 0.0f ? 1.0f : -1.0f;
            moveTo(OWN_GOAL_X_MM, side * SIDE_WALL_TARGET_Y_MM,
                   180.0f - side * 45.0f,
                   0.8f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }

        case RobotGoal::spinKick: {
            const float goalHeading = headingToOpponentGoal();
            if (!spinKickTracking) {
                spinKickLastHeading = robotPose.headingDeg;
                spinKickAccumulatedDeg = 0.0f;
                spinKickTracking = true;
            }
            spinKickAccumulatedDeg +=
                fabsf(angleError(robotPose.headingDeg, spinKickLastHeading));
            spinKickLastHeading = robotPose.headingDeg;
            if (!goalDriveStarted &&
                ballState.ballPossession == BallPossession::mePossession &&
                spinKickAccumulatedDeg >= 360.0f &&
                fabsf(angleError(goalHeading, robotPose.headingDeg)) <=
                    SPIN_KICK_HEADING_TOLERANCE_DEG) {
                goalDriveStarted = true;
            }
            if (goalDriveStarted) {
                moveTo(OPPONENT_GOAL_X_MM, OPPONENT_GOAL_Y_MM, goalHeading,
                       BALL_RAM_SPEED, BALL_RAM_ACCEL_LIMIT, ROTATION_MAX_SPEED,
                       ROTATION_ACCEL_LIMIT);
            } else {
                stopAllDriveMotors();
                drive(0.0f, 0.0f, ROTATION_MAX_SPEED);
            }
            break;
        }

        case RobotGoal::kick: {
            // Ram: lined up behind the ball, drive full speed at the goal centre.
            if (!ball.valid) {
                stopMotion();
                break;
            }
            const float headingError = angleError(headingToOpponentGoal(),
                                                  robotPose.headingDeg);
            const float ramHeading = robotPose.headingDeg +
                constrain(headingError * RAM_HEADING_GAIN,
                          -RAM_HEADING_ERROR_MAX_DEG, RAM_HEADING_ERROR_MAX_DEG);
            moveTo(OPPONENT_GOAL_X_MM, OPPONENT_GOAL_Y_MM, ramHeading,
                   ROBOT_MAX_SPEED, ACCEL_LIMIT,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }

        case RobotGoal::pass: {
            if (!remotePose.valid) {
                stopMotion();
                break;
            }
            const float passHeading = headingTo(remotePose.xMm, remotePose.yMm);
            moveTo(robotPose.xMm, robotPose.yMm,
                   passHeading, 0.0f, 0.0f,
                   ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            // Passing needs the kicker, which is disabled: only face the partner.
            break;
        }

        case RobotGoal::backOff:
                        moveTo(0.0f, 0.0f,
                                     headingToOpponentGoal(),
                   0.7f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;

        case RobotGoal::defendBall: {
            if (!ball.valid) {
                stopAllDriveMotors();
                break;
            }
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
                stopMotion();
                break;
            }
            moveTo(clampDefenceX(opponent.xMm), clampDefenceY(opponent.yMm),
                   headingTo(opponent.xMm, opponent.yMm),
                   0.9f, ACCEL_LIMIT, ROTATION_MAX_SPEED, ROTATION_ACCEL_LIMIT);
            break;
        }

        case RobotGoal::searchBall: {
            const float distanceToCenter =
                sqrtf(robotPose.xMm * robotPose.xMm +
                      robotPose.yMm * robotPose.yMm);
            if (!searchSettled && distanceToCenter <= SEARCH_SPIN_RADIUS_MM) {
                searchSettled = true;
            } else if (searchSettled &&
                       distanceToCenter > SEARCH_RESUME_RADIUS_MM) {
                searchSettled = false;
            }

            if (!searchSettled) {
                moveTo(0.0f, 0.0f, robotPose.headingDeg,
                       0.25f, 1.0f, 0.0f, ROTATION_ACCEL_LIMIT);
            } else {
                stopAllDriveMotors();
                drive(0.0f, 0.0f, 0.15f);
            }
            break;
        }

        case RobotGoal::awayBorders:
            // Handled above so it cannot be pre-empted by another action.
            break;
    }
    updateDribblerPolicy();
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
    if (opponent1.valid == false) {
        opponent1State = OpponentState::damaged;
    } else {
        if (opponent1State == OpponentState::hidingBall) { // checks if hiding ball first
            if (isInOpponentGoalBox(opponent1)) {
                opponent1State = OpponentState::goalie;
            } else if (opponent1.xMm >= -MIDDLE_ZONE_X) {
                opponent1State = OpponentState::shooting;
            } 
        } else {
            if (isInOpponentGoalBox(opponent1)) {
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
            if (isInOpponentGoalBox(opponent2)) {
                opponent2State = OpponentState::goalie;
            } else if (opponent2.xMm >= -MIDDLE_ZONE_X) {
                opponent2State = OpponentState::shooting;
            } 
        } else {
            if (isInOpponentGoalBox(opponent2)) {
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
    if (localState.robotState == RobotState::damaged) {
        localState.robotGoal = RobotGoal::none; // DAMAGED
        return;
    }

    const bool attacking = localState.robotState == RobotState::attacking;
    const float earlyMargin = attacking
        ? ATTACKER_BORDER_EARLY_MARGIN_MM : BORDER_EARLY_MARGIN_MM;
    const float exitMargin = attacking
        ? ATTACKER_BORDER_EXIT_MARGIN_MM : BORDER_EXIT_MARGIN_MM;
    const float margin = earlyMargin +
        (localState.robotGoal == RobotGoal::awayBorders ? exitMargin : 0.0f);
    const float lookaheadMm = currentMoveProfile.active
        ? currentMoveProfile.speed * ROBOT_LINEAR_SPEED_MM_S *
          (attacking ? ATTACKER_BORDER_LOOKAHEAD_S : BORDER_LOOKAHEAD_S) : 0.0f;
    const float directionRad = currentMoveProfile.movementDirectionDeg * PI / 180.0f;
    if (isNearBorder(robotPose.xMm, robotPose.yMm, margin) ||
        isNearBorder(robotPose.xMm + lookaheadMm * cosf(directionRad),
                     robotPose.yMm + lookaheadMm * sinf(directionRad), margin)) {
        localState.robotGoal = RobotGoal::awayBorders; // FIRST CHECK BORDERS
        return;
    }

    if (localState.robotState == RobotState::attacking) {
        const RobotGoal currentGoal = localState.robotGoal;
        const BallPossession possession = ballState.ballPossession;

        const bool hasBallForAttack =
            possession == BallPossession::mePossession ||
            possession == BallPossession::front;
        if (isBallInDefenceBox() && !hasBallForAttack) {
            localState.robotGoal = RobotGoal::backOff;
            return;
        }

        if (possession == BallPossession::himPossession) {
            localState.robotGoal = RobotGoal::defendBall;
            return;
        }
        if (possession == BallPossession::theirPossession1) {
            localState.robotGoal = RobotGoal::interceptBall1;
            return;
        }
        if (possession == BallPossession::theirPossession2) {
            localState.robotGoal = RobotGoal::interceptBall2;
            return;
        }

        if (currentGoal == RobotGoal::scoring) {
            if (ball.valid &&
                (possession == BallPossession::mePossession ||
                 possession == BallPossession::front)) {
                localState.robotGoal = RobotGoal::scoring;
                return;
            }
        }

        if (isBallInGoalPushZone() &&
            (possession == BallPossession::mePossession ||
             possession == BallPossession::front)) {
            localState.robotGoal = RobotGoal::scoring;
            return;
        }

        /*
        // The behind-ball route has its own translation stage. Side-zone
        // handling does not redirect this route into hideBall.
        const bool behindRoute = isBehindBallRoute(currentGoal);
        if (behindRoute && currentGoal != RobotGoal::scoring &&
            ((possession == BallPossession::front) ||
             (possession == BallPossession::mePossession))) {
            localState.robotGoal = isAtKickLine()
                ? RobotGoal::kick : RobotGoal::dribbleForward;
            return;
        }

        if (possession == BallPossession::mePossession) {
            if (currentGoal == RobotGoal::hideBall) {
                localState.robotGoal = hideBallReached()
                    ? RobotGoal::kick
                    : RobotGoal::hideBall;
            } else if (currentGoal == RobotGoal::kick && ball.valid &&
                       ball.distanceCm <= BALL_TARGET_DISTANCE_CM) {
                localState.robotGoal = RobotGoal::kick;
            } else if (isAtKickLine()) {
                localState.robotGoal = RobotGoal::kick;
            } else if (isSideRoute()) {
                localState.robotGoal = hideBallReached()
                    ? RobotGoal::kick
                    : RobotGoal::hideBall;
            } else {
                localState.robotGoal = RobotGoal::dribbleForward;
            }
            return;
        }

        if (possession == BallPossession::front) {
            if (currentGoal == RobotGoal::hideBall) {
                localState.robotGoal = hideBallReached()
                    ? RobotGoal::kick
                    : RobotGoal::hideBall;
            } else if (currentGoal == RobotGoal::getBallDribble) {
                localState.robotGoal = isAtKickLine()
                    ? RobotGoal::kick
                    : isSideRoute()
                        ? (hideBallReached() ? RobotGoal::kick
                                             : RobotGoal::hideBall)
                        : RobotGoal::dribbleForward;
            } else if (currentGoal == RobotGoal::kick && ball.valid &&
                       ball.distanceCm <= BALL_TARGET_DISTANCE_CM) {
                localState.robotGoal = RobotGoal::kick;
            } else if (isAtKickLine()) {
                localState.robotGoal = RobotGoal::kick;
            } else if (currentGoal == RobotGoal::dribbleForward ||
                       currentGoal == RobotGoal::pushForward) {
                localState.robotGoal = RobotGoal::dribbleForward;
            } else {
                // Front means the ball has reached the dribbler; keep
                // collecting until the localized possession state confirms it.
                localState.robotGoal = RobotGoal::getBallDribble;
            }
            return;
        }

        if (currentGoal == RobotGoal::kick) {
            // Ball has been kicked or lost.
            if (ball.valid) {
                localState.robotGoal = ball.xMm > 0.0f
                    ? RobotGoal::getBallPush : RobotGoal::getBallDribble;
            } else {
                localState.robotGoal = RobotGoal::searchBall;
            }
            return;
        }
        if (currentGoal == RobotGoal::hideBall && ball.valid &&
            ball.distanceCm <= BALL_TARGET_DISTANCE_CM) {
            localState.robotGoal = RobotGoal::hideBall;
            return;
        }
        if (currentGoal == RobotGoal::pushForward) {
            if (ball.valid) {
                localState.robotGoal = ball.xMm > 0.0f
                    ? RobotGoal::getBallPush : RobotGoal::getBallDribble;
            } else {
                localState.robotGoal = RobotGoal::searchBall;
            }
            return;
        }
        if (currentGoal == RobotGoal::dribbleForward && ball.valid) {
            localState.robotGoal = RobotGoal::getBallDribble;
            return;
        }

        */

        // Orbit the ball facing the goal; ram once the ball is close and in line.
        // Ramming continues (with hysteresis) while the ball stays close and ahead.
        if (!ball.valid) {
            localState.robotGoal = RobotGoal::searchBall;
        } else {
            const bool keepRamming = currentGoal == RobotGoal::kick &&
                ball.distanceCm <= RAM_KEEP_DISTANCE_CM &&
                isLinedUpBehindBall(RAM_KEEP_LINEUP_DEG);
            const bool startRamming = ball.distanceCm <= RAM_START_DISTANCE_CM &&
                isLinedUpBehindBall(RAM_START_LINEUP_DEG);
            localState.robotGoal = (keepRamming || startRamming)
                ? RobotGoal::kick : RobotGoal::getBallPush;
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
