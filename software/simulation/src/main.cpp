#include <SFML/Graphics.hpp>
#include <cmath>
#include <string>
#include <algorithm>
#include <iostream>

#include "subsystems/robot_state.h"

// Simple 2D RCJ Open Soccer simulation
// - Units in this file are millimetres (mm) for field/robot constants
// - The graphics scale converts mm -> pixels (PX_PER_MM)

// Field constants (from user)
constexpr float FIELD_WIDTH_MM = 1430.0f;   // x dimension (mm)
constexpr float FIELD_HEIGHT_MM = 1820.0f;  // y dimension (mm)
constexpr float WHITE_LINE_THICKNESS_MM = 50.0f;
constexpr float WHITE_LINE_INSET_FROM_WALL_MM = 250.0f; // distance from outer wall to the white line
constexpr float GOAL_WIDTH_MM = 450.0f;
constexpr float GOAL_DEPTH_MM = 74.0f; // how far the goal extends inward (visual)

// Robot (configurable)
constexpr float ROBOT_DIAMETER_MM = 180.0f; // default robot diameter (changeable)
constexpr float DRIBBLER_WIDTH_MM = 40.0f;  // frontal dribbler width
constexpr float DRIBBLER_DEPTH_MM = 12.0f;  // how far it projects

// Ball (40 mm diameter golf-ball sized)
constexpr float BALL_DIAMETER_MM = 40.0f;
constexpr float BALL_RADIUS_MM = BALL_DIAMETER_MM / 2.0f;

// Ball physics tuning (units: mm, mm/s, seconds)
constexpr float BALL_FRICTION_PER_S = 2.0f;   // exponential rolling-decay coefficient (1/s)
constexpr float BALL_WALL_RESTITUTION = 0.6f; // fraction of speed kept when bouncing off walls/goals
constexpr float BALL_MAX_SPEED_MM_S = 5000.0f; // safety cap: prevents tunneling through goal boxes
constexpr float ROBOT_PUSH_FACTOR = 0.65f;    // fraction of the robot's closing speed imparted to the ball

// Rendering
float PX_PER_MM = 0.35f; // default scale (pixels per millimetre). Adjust for window size.
const int WINDOW_MARGIN_PX = 40;
const int HUD_PANEL_WIDTH_PX = 320; // fixed HUD panel width on the right side

// Helper conversions
inline float mmToPx(float mm) { return mm * PX_PER_MM; }

struct AxisAlignedRect {
    float left;
    float top;
    float width;
    float height;
};

// Goals are solid rectangular obstacles in simulation coordinates. Keeping
// these bounds in millimetres makes their collision independent of rendering
// scale and matches the rectangles drawn later in main().
const AxisAlignedRect TOP_GOAL_BOUNDS = {
    (FIELD_WIDTH_MM - GOAL_WIDTH_MM) / 2.0f,
    WHITE_LINE_INSET_FROM_WALL_MM + WHITE_LINE_THICKNESS_MM / 2.0f - GOAL_DEPTH_MM,
    GOAL_WIDTH_MM,
    GOAL_DEPTH_MM
};
const AxisAlignedRect BOTTOM_GOAL_BOUNDS = {
    (FIELD_WIDTH_MM - GOAL_WIDTH_MM) / 2.0f,
    FIELD_HEIGHT_MM - WHITE_LINE_INSET_FROM_WALL_MM - WHITE_LINE_THICKNESS_MM / 2.0f,
    GOAL_WIDTH_MM,
    GOAL_DEPTH_MM
};

float clampFloat(float value, float minimum, float maximum) {
    return std::max(minimum, std::min(maximum, value));
}

void resolveCircleAgainstRect(sf::Vector2f& position, float radius, const AxisAlignedRect& rect) {
    const float right = rect.left + rect.width;
    const float bottom = rect.top + rect.height;
    const float closestX = clampFloat(position.x, rect.left, right);
    const float closestY = clampFloat(position.y, rect.top, bottom);
    const float dx = position.x - closestX;
    const float dy = position.y - closestY;
    const float distanceSquared = dx * dx + dy * dy;

    if (distanceSquared >= radius * radius) {
        return;
    }

    if (distanceSquared > 0.0001f) {
        const float distance = std::sqrt(distanceSquared);
        const float pushDistance = radius - distance;
        position.x += dx / distance * pushDistance;
        position.y += dy / distance * pushDistance;
        return;
    }

    // The circle centre is inside the goal. Push it through its nearest face.
    const float toLeft = position.x - rect.left;
    const float toRight = right - position.x;
    const float toTop = position.y - rect.top;
    const float toBottom = bottom - position.y;
    const float nearestFace = std::min(std::min(toLeft, toRight), std::min(toTop, toBottom));

    if (nearestFace == toLeft) position.x = rect.left - radius;
    else if (nearestFace == toRight) position.x = right + radius;
    else if (nearestFace == toTop) position.y = rect.top - radius;
    else position.y = bottom + radius;
}

void constrainToFieldAndGoals(sf::Vector2f& position, float radius) {
    position.x = clampFloat(position.x, radius, FIELD_WIDTH_MM - radius);
    position.y = clampFloat(position.y, radius, FIELD_HEIGHT_MM - radius);
    resolveCircleAgainstRect(position, radius, TOP_GOAL_BOUNDS);
    resolveCircleAgainstRect(position, radius, BOTTOM_GOAL_BOUNDS);
}

void moveWithGoalCollision(sf::Vector2f& position, const sf::Vector2f& target, float radius) {
    const sf::Vector2f delta = target - position;
    const float distance = std::sqrt(delta.x * delta.x + delta.y * delta.y);
    // Sweep in short increments so a fast robot or mouse drag cannot step
    // completely through a shallow goal in one frame.
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / std::max(1.0f, radius * 0.5f))));
    const sf::Vector2f step = delta / static_cast<float>(steps);

    for (int i = 0; i < steps; ++i) {
        position += step;
        constrainToFieldAndGoals(position, radius);
    }
}

// Ball position is only ever clamped to the field walls. Dragging/dropping the
// ball must place it exactly where the cursor is (even over/behind a goal);
// goal collisions only apply to the ball while it is rolling (moveBall below).
void clampBallToField(sf::Vector2f& position) {
    position.x = clampFloat(position.x, BALL_RADIUS_MM, FIELD_WIDTH_MM - BALL_RADIUS_MM);
    position.y = clampFloat(position.y, BALL_RADIUS_MM, FIELD_HEIGHT_MM - BALL_RADIUS_MM);
}

// Ball drag: follow the mouse exactly (clamped to the field walls). No goal
// collision here, so the ball never snaps to a goal surface while dragging.
void moveBallWithCollision(sf::Vector2f& position, const sf::Vector2f& target) {
    position = target;
    clampBallToField(position);
}

// Keep the ball out of the goal boxes themselves (no scoring possible yet):
// if the ball's centre is ever inside a goal box (e.g. shoved over the mouth
// by the robot), return it to that goal's mouth. This is called only from the
// physics step, never while the user is dragging, so it cannot snap a ball to
// the goal surface as it is being dropped in the open field.
void containBallInGoalBoxes(sf::Vector2f& position) {
    if (position.x > BOTTOM_GOAL_BOUNDS.left && position.x < BOTTOM_GOAL_BOUNDS.left + BOTTOM_GOAL_BOUNDS.width &&
        position.y > BOTTOM_GOAL_BOUNDS.top && position.y < BOTTOM_GOAL_BOUNDS.top + BOTTOM_GOAL_BOUNDS.height) {
        position.y = BOTTOM_GOAL_BOUNDS.top - BALL_RADIUS_MM;
    }
    if (position.x > TOP_GOAL_BOUNDS.left && position.x < TOP_GOAL_BOUNDS.left + TOP_GOAL_BOUNDS.width &&
        position.y > TOP_GOAL_BOUNDS.top && position.y < TOP_GOAL_BOUNDS.top + TOP_GOAL_BOUNDS.height) {
        position.y = TOP_GOAL_BOUNDS.top + TOP_GOAL_BOUNDS.height + BALL_RADIUS_MM;
    }
}

void resolveBallAgainstRect(sf::Vector2f& position, sf::Vector2f& velocity, const AxisAlignedRect& rect) {
    const float right = rect.left + rect.width;
    const float bottom = rect.top + rect.height;

    // Left face (ball just to the left of the face)
    if (position.x > rect.left - BALL_RADIUS_MM && position.x < rect.left &&
        position.y > rect.top && position.y < bottom) {
        position.x = rect.left - BALL_RADIUS_MM;
        velocity.x = std::abs(velocity.x) * BALL_WALL_RESTITUTION;
    }
    // Right face (ball just to the right of the face)
    if (position.x > right && position.x < right + BALL_RADIUS_MM &&
        position.y > rect.top && position.y < bottom) {
        position.x = right + BALL_RADIUS_MM;
        velocity.x = -std::abs(velocity.x) * BALL_WALL_RESTITUTION;
    }
    // Top face (ball just above the face)
    if (position.y > rect.top - BALL_RADIUS_MM && position.y < rect.top &&
        position.x > rect.left && position.x < right) {
        position.y = rect.top - BALL_RADIUS_MM;
        velocity.y = std::abs(velocity.y) * BALL_WALL_RESTITUTION;
    }
    // Bottom face (ball just below the face)
    if (position.y > bottom && position.y < bottom + BALL_RADIUS_MM &&
        position.x > rect.left && position.x < right) {
        position.y = bottom + BALL_RADIUS_MM;
        velocity.y = -std::abs(velocity.y) * BALL_WALL_RESTITUTION;
    }
}

void moveBall(sf::Vector2f& position, sf::Vector2f& velocity, float dt) {
    // Rolling friction
    velocity *= std::exp(-BALL_FRICTION_PER_S * dt);

    // Safety cap so an extreme robot push (high moveSpeedScale) cannot launch
    // the ball fast enough to skip past the goal boxes in one frame.
    const float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    if (speed > BALL_MAX_SPEED_MM_S) {
        velocity *= BALL_MAX_SPEED_MM_S / speed;
    }

    // Sweep in short increments (same idea as moveWithGoalCollision) so a fast
    // ball bounces off walls/goals instead of tunnelling straight through the
    // shallow goal boxes.
    const float maxStepDist = BALL_RADIUS_MM * 0.5f;
    float remaining = dt;
    while (remaining > 0.0f) {
        const float currentSpeed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
        float stepTime = remaining;
        if (currentSpeed > 0.0001f) {
            stepTime = std::min(remaining, maxStepDist / currentSpeed);
        }
        position += velocity * stepTime;
        remaining -= stepTime;

        // Outer walls (reflect velocity, keeping a bit of energy)
        if (position.x < BALL_RADIUS_MM) {
            position.x = BALL_RADIUS_MM;
            velocity.x = std::abs(velocity.x) * BALL_WALL_RESTITUTION;
        } else if (position.x > FIELD_WIDTH_MM - BALL_RADIUS_MM) {
            position.x = FIELD_WIDTH_MM - BALL_RADIUS_MM;
            velocity.x = -std::abs(velocity.x) * BALL_WALL_RESTITUTION;
        }
        if (position.y < BALL_RADIUS_MM) {
            position.y = BALL_RADIUS_MM;
            velocity.y = std::abs(velocity.y) * BALL_WALL_RESTITUTION;
        } else if (position.y > FIELD_HEIGHT_MM - BALL_RADIUS_MM) {
            position.y = FIELD_HEIGHT_MM - BALL_RADIUS_MM;
            velocity.y = -std::abs(velocity.y) * BALL_WALL_RESTITUTION;
        }

        // Goals (treated as solid boxes, consistent with the robot)
        resolveBallAgainstRect(position, velocity, TOP_GOAL_BOUNDS);
        resolveBallAgainstRect(position, velocity, BOTTOM_GOAL_BOUNDS);
    }
}

// Dribbler physics: when the robot code requests the dribbler (dribblerShouldRun)
// and the ball touches the frontal dribbler bar, hold the ball against the bar
// so it moves and rotates with the robot. Once held, the ball stays held (re-
// pinned to the bar every frame) until the dribbler is switched off or the user
// grabs the ball. Returns true while the ball is held; the caller should then
// skip rolling/robot-body ball physics.
bool updateBallDribbling(sf::Vector2f& ballPosMm, sf::Vector2f& ballVelMmS,
                         bool ballHeld,
                         const sf::Vector2f& robotPos, float robotDiameterMm,
                         float headingDeg, const sf::Vector2f& robotVelMmS,
                         bool dribblerShouldRun) {
    if (!dribblerShouldRun) {
        return false;
    }
    const float radius = robotDiameterMm * 0.5f;
    // Dribbler bar centre sits just past the front of the robot body
    // (matches how the dribbler is drawn in Robot::draw).
    const float barCenterOffset = radius + 1.0f + DRIBBLER_DEPTH_MM * 0.5f;
    const float halfDepth = DRIBBLER_DEPTH_MM * 0.5f;
    const float halfWidth = DRIBBLER_WIDTH_MM * 0.5f;

    // Robot front direction (0 deg = up / negative screen-y)
    const float angleRad = (headingDeg - 90.0f) * 3.14159265f / 180.0f;
    const sf::Vector2f frontVec(std::cos(angleRad), std::sin(angleRad));

    // Ball position relative to the robot, in the dribbler bar's local frame
    const sf::Vector2f toBall = ballPosMm - robotPos;
    const float forward = toBall.x * frontVec.x + toBall.y * frontVec.y;
    const float lateral = std::abs(toBall.x * frontVec.y - toBall.y * frontVec.x);
    const float u = forward - barCenterOffset;

    // The ball touches the bar when its centre is within a ball radius of the
    // bar rectangle (front face, or the side edges as it is scooped up).
    const bool touchesBar =
        std::abs(u) <= halfDepth + BALL_RADIUS_MM &&
        lateral <= halfWidth + BALL_RADIUS_MM;
    if (!ballHeld && !touchesBar) {
        return false;
    }

    // Hold the ball against the front of the bar and carry the robot's motion
    // over so a released ball keeps rolling naturally.
    ballVelMmS = robotVelMmS;
    ballPosMm = robotPos + frontVec * (barCenterOffset + halfDepth + BALL_RADIUS_MM);
    return true;
}

// The robot is far heavier than the ball, so on contact the ball is pushed out
// of the overlap and picks up a fraction of the robot's closing speed.
void resolveRobotBallCollision(sf::Vector2f& ballPos, sf::Vector2f& ballVel,
                               const sf::Vector2f& robotPos, float robotRadius,
                               const sf::Vector2f& robotVel) {
    const float minDist = robotRadius + BALL_RADIUS_MM;
    const sf::Vector2f toBall = ballPos - robotPos;
    const float distSq = toBall.x * toBall.x + toBall.y * toBall.y;
    if (distSq >= minDist * minDist) {
        return;
    }

    const float dist = std::sqrt(distSq);
    sf::Vector2f normal(0.0f, -1.0f);
    if (dist > 0.0001f) {
        normal = toBall / dist;
    }

    // Push the ball out to the robot's edge so they never overlap
    ballPos = robotPos + normal * minDist;

    // Only transfer momentum when the ball is approaching the robot
    const float relativeVelN = (ballVel.x - robotVel.x) * normal.x + (ballVel.y - robotVel.y) * normal.y;
    if (relativeVelN < 0.0f) {
        ballVel -= normal * (relativeVelN * ROBOT_PUSH_FACTOR);
    }
}

struct Robot {
    sf::Vector2f pos; // in mm
    float headingDeg; // 0 = up (toward negative y in screen coords)
    float diameterMm;

    Robot(): pos(FIELD_WIDTH_MM/2.0f, FIELD_HEIGHT_MM/2.0f), headingDeg(0.0f), diameterMm(ROBOT_DIAMETER_MM) {}

    void draw(sf::RenderTarget& rt) const {
        float r_px = mmToPx(diameterMm) / 2.0f;
        sf::CircleShape body(r_px);
        body.setOrigin(sf::Vector2f(r_px, r_px));
        body.setPosition(sf::Vector2f(mmToPx(pos.x) + WINDOW_MARGIN_PX, mmToPx(pos.y) + WINDOW_MARGIN_PX));
        body.setFillColor(sf::Color(200, 200, 220));
        body.setOutlineThickness(2.0f);
        body.setOutlineColor(sf::Color::Black);
        rt.draw(body);

        // draw heading indicator
        float angleRad = (headingDeg - 90.0f) * 3.14159265f / 180.0f; // convert to screen angle
        sf::Vector2f dir(std::cos(angleRad), std::sin(angleRad));
        sf::VertexArray line(sf::PrimitiveType::Lines, 2);
        line[0].position = body.getPosition();
        line[0].color = sf::Color::Black;
        line[1].position = body.getPosition() + dir * (r_px * 1.1f);
        line[1].color = sf::Color::Red;
        rt.draw(line);

        // draw frontal dribbler as a small rectangle in front
        float dribblerW_px = mmToPx(DRIBBLER_WIDTH_MM);
        float dribblerD_px = mmToPx(DRIBBLER_DEPTH_MM);
        sf::RectangleShape drib(sf::Vector2f(dribblerW_px, dribblerD_px));
        drib.setOrigin(sf::Vector2f(dribblerW_px/2.0f, dribblerD_px));
        // position the dribbler at robot front
        sf::Vector2f front = body.getPosition() + dir * (r_px + dribblerD_px*0.5f + 1.0f);
        drib.setPosition(front);
        drib.setRotation(sf::degrees(headingDeg));
        drib.setFillColor(sf::Color(120,120,120));
        rt.draw(drib);
    }
};

int main() {
    // Compute an appropriate PX_PER_MM to maximize field size on the current screen while leaving a safety margin
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    unsigned int screenW = desktop.size.x;
    unsigned int screenH = desktop.size.y;
    const int EXTRA_MARGIN_W = 80; // extra safety margin beyond WINDOW_MARGIN_PX
    const int EXTRA_MARGIN_H = 100;
    // Reserve space horizontally for the HUD panel + an extra inner margin
    float reservedForHudPx = static_cast<float>(HUD_PANEL_WIDTH_PX + WINDOW_MARGIN_PX);
    float max_px_per_mm_w = float((int)screenW - 2*WINDOW_MARGIN_PX - EXTRA_MARGIN_W - (int)reservedForHudPx) / FIELD_WIDTH_MM;
    float max_px_per_mm_h = float((int)screenH - 2*WINDOW_MARGIN_PX - EXTRA_MARGIN_H) / FIELD_HEIGHT_MM;
    float chosen = std::min(max_px_per_mm_w, max_px_per_mm_h);
    // clamp to reasonable range
    PX_PER_MM = std::max(0.05f, std::min(chosen, 5.0f));

    // Compute window size in pixels from field dims using chosen scale.
    // Layout: [ left margin | field px | inner margin | HUD panel | right margin ]
    int winW = static_cast<int>(std::round(mmToPx(FIELD_WIDTH_MM))) + 3*WINDOW_MARGIN_PX + HUD_PANEL_WIDTH_PX;
    int winH = static_cast<int>(std::round(mmToPx(FIELD_HEIGHT_MM))) + 2*WINDOW_MARGIN_PX;

    sf::VideoMode mode(sf::Vector2u(static_cast<unsigned int>(winW), static_cast<unsigned int>(winH)), 32);
    sf::RenderWindow window(mode, sf::String("RCJ Open Soccer - 2D Simulation"));
    window.setFramerateLimit(60);

    std::cout << "Screen: " << screenW << "x" << screenH << ", PX_PER_MM=" << PX_PER_MM << "\n";

    Robot robot;

    // Create static shapes
    // full green area
    sf::RectangleShape greenBg(sf::Vector2f(mmToPx(FIELD_WIDTH_MM), mmToPx(FIELD_HEIGHT_MM)));
    greenBg.setPosition(sf::Vector2f(WINDOW_MARGIN_PX, WINDOW_MARGIN_PX));
    greenBg.setFillColor(sf::Color(50, 140, 60)); // green carpet
    greenBg.setOutlineThickness(2.0f);
    greenBg.setOutlineColor(sf::Color::Black);

    // walls (visual) - outer border drawn by margin+thin rects
    float outerW_px = mmToPx(FIELD_WIDTH_MM);
    float outerH_px = mmToPx(FIELD_HEIGHT_MM);

    // white lines (drawn as 4 rectangles inset by WHITE_LINE_INSET_FROM_WALL_MM)
    float inset_px = mmToPx(WHITE_LINE_INSET_FROM_WALL_MM);
    float lineThickness_px = mmToPx(WHITE_LINE_THICKNESS_MM);

    // top line rectangle
    sf::RectangleShape topLine(sf::Vector2f(outerW_px - 2*inset_px, lineThickness_px));
    topLine.setPosition(sf::Vector2f(WINDOW_MARGIN_PX + inset_px, WINDOW_MARGIN_PX + inset_px - lineThickness_px/2.0f));
    topLine.setFillColor(sf::Color::White);

    // bottom line
    sf::RectangleShape bottomLine(sf::Vector2f(outerW_px - 2*inset_px, lineThickness_px));
    bottomLine.setPosition(sf::Vector2f(WINDOW_MARGIN_PX + inset_px, WINDOW_MARGIN_PX + outerH_px - inset_px - lineThickness_px/2.0f));
    bottomLine.setFillColor(sf::Color::White);

    // left line
    sf::RectangleShape leftLine(sf::Vector2f(lineThickness_px, outerH_px - 2*inset_px));
    leftLine.setPosition(sf::Vector2f(WINDOW_MARGIN_PX + inset_px - lineThickness_px/2.0f, WINDOW_MARGIN_PX + inset_px));
    leftLine.setFillColor(sf::Color::White);

    // right line
    sf::RectangleShape rightLine(sf::Vector2f(lineThickness_px, outerH_px - 2*inset_px));
    rightLine.setPosition(sf::Vector2f(WINDOW_MARGIN_PX + outerW_px - inset_px - lineThickness_px/2.0f, WINDOW_MARGIN_PX + inset_px));
    rightLine.setFillColor(sf::Color::White);

    // Goals: centered on short sides (top and bottom). We'll draw simple goal rectangles (opening width GOAL_WIDTH_MM and depth GOAL_DEPTH_MM)
    float goalW_px = mmToPx(GOAL_WIDTH_MM);
    float goalD_px = mmToPx(GOAL_DEPTH_MM);
    float centerX_px = WINDOW_MARGIN_PX + outerW_px/2.0f;

    // Goals begin at the inner edge of each white line and extend away from
    // the centre of the field.
    // Compute the inner edge of the white top line in pixels.
    float whiteTopInnerY_px = WINDOW_MARGIN_PX + inset_px + (lineThickness_px/2.0f);
    float topGoalInnerY_px = whiteTopInnerY_px;
    sf::RectangleShape topGoal(sf::Vector2f(goalW_px, goalD_px));
    topGoal.setOrigin(sf::Vector2f(goalW_px/2.0f, goalD_px)); // inner edge is the bottom edge
    topGoal.setPosition(sf::Vector2f(centerX_px, topGoalInnerY_px));
    topGoal.setFillColor(sf::Color(255,255,0)); // CMYK Yellow -> RGB (255,255,0)
    topGoal.setOutlineThickness(2.0f);
    topGoal.setOutlineColor(sf::Color::Black);

    // Bottom goal (symmetrical)
    float whiteBottomInnerY_px = WINDOW_MARGIN_PX + outerH_px - inset_px - (lineThickness_px/2.0f);
    float bottomGoalInnerY_px = whiteBottomInnerY_px;
    sf::RectangleShape bottomGoal(sf::Vector2f(goalW_px, goalD_px));
    bottomGoal.setOrigin(sf::Vector2f(goalW_px/2.0f, 0.0f)); // inner edge is the top edge
    bottomGoal.setPosition(sf::Vector2f(centerX_px, bottomGoalInnerY_px));
    bottomGoal.setFillColor(sf::Color(0,255,255)); // CMYK Cyan -> RGB (0,255,255)
    bottomGoal.setOutlineThickness(2.0f);
    bottomGoal.setOutlineColor(sf::Color::Black);

    // Basic control variables
    const float ROBOT_SPEED_MM_S = 220.0f; // forward speed mm per second
    const float ROTATION_SPEED_DEG_S = 120.0f;

    sf::Clock clock;
    bool dragging = false;
    sf::Vector2f dragOffsetMm(0.f, 0.f);

    // Robot/robotcode control
    bool robotEnabled = true; // initial state
    extern void robot_init();
    extern void robot_stop();
    extern void sim_set_millis(unsigned long ms);

    // Start robot code thread (it will call setup() and then loop())
    robot_init();
    // set the robot_state flag so robot code can read whether it should be running
    robotCurrentlyRunning = robotEnabled;

    // Ball entity
    sf::Vector2f ballPosMm(FIELD_WIDTH_MM*0.25f, FIELD_HEIGHT_MM*0.5f);
    sf::Vector2f ballVelMmS(0.0f, 0.0f); // ball velocity in mm/s (rolled/pushed around by physics)
    bool draggingBall = false;
    sf::Vector2f ballDragOffsetMm(0.f, 0.f);
    bool ballHeld = false; // true while the dribbler is holding the ball

    unsigned long simMs = 0;
    const unsigned long SIM_MS_PER_FRAME = 16; // ~60Hz sim time step (16ms)

    // Movement scaling controls (HUD adjustable)
    float moveSpeedScale = 8.0f;   // user-editable multiplier for linear speed (default 8.0)
    float rotSpeedScale = 8.0f;    // user-editable multiplier for rotation speed (default 8.0)
    std::string moveSpeedStr = std::to_string(moveSpeedScale);
    std::string rotSpeedStr = std::to_string(rotSpeedScale);
    bool editMove = false;
    bool editRot = false;

    // Max physical speeds used by simulator (mm/s and deg/s)
    const float SIM_MAX_LINEAR_MM_S = 220.0f; // base max linear speed
    const float SIM_MAX_ROT_DEG_S = 120.0f;   // base max rotation speed

    while (window.isOpen()) {
        // Event handling (SFML3 uses std::optional<Event>)
        while (auto evOpt = window.pollEvent()) {
            const auto &ev = *evOpt;
            if (ev.is<sf::Event::Closed>()) {
                window.close();
            }
            if (ev.is<sf::Event::KeyPressed>()) {
                const auto *kp = ev.getIf<sf::Event::KeyPressed>();
                if (kp) {
                    if (kp->code == sf::Keyboard::Key::Escape) window.close();
                    if (kp->code == sf::Keyboard::Key::R) {
                        robot.pos = sf::Vector2f(FIELD_WIDTH_MM/2.0f, FIELD_HEIGHT_MM/2.0f);
                        robot.headingDeg = 0.0f;
                    }
                    // Handle input editing keys
                    if (kp->code == sf::Keyboard::Key::Backspace) {
                        if (editMove && !moveSpeedStr.empty()) moveSpeedStr.pop_back();
                        if (editRot && !rotSpeedStr.empty()) rotSpeedStr.pop_back();
                    }
                    if (kp->code == sf::Keyboard::Key::Enter) {
                        if (editMove) {
                            try { moveSpeedScale = std::stof(moveSpeedStr); } catch(...) {}
                            editMove = false;
                        }
                        if (editRot) {
                            try { rotSpeedScale = std::stof(rotSpeedStr); } catch(...) {}
                            editRot = false;
                        }
                    }
                    if (kp->code == sf::Keyboard::Key::Escape) {
                        // cancel editing
                        if (editMove) { moveSpeedStr = std::to_string(moveSpeedScale); editMove = false; }
                        if (editRot) { rotSpeedStr = std::to_string(rotSpeedScale); editRot = false; }
                    }
                }
            }

            if (ev.is<sf::Event::TextEntered>()) {
                const auto *te = ev.getIf<sf::Event::TextEntered>();
                if (te) {
                    uint32_t uni = te->unicode;
                    // allow digits, dot, minus
                    if (uni == 8) { // backspace handled in KeyPressed, ignore
                    } else if (uni < 128) {
                        char c = static_cast<char>(uni);
                        if ((c >= '0' && c <= '9') || c == '.' || c == '-') {
                            if (editMove) moveSpeedStr.push_back(c);
                            if (editRot) rotSpeedStr.push_back(c);
                        }
                    }
                }
            }

            // Mouse: start dragging if left-click on robot or ball
            if (ev.is<sf::Event::MouseButtonPressed>()) {
                const auto *mb = ev.getIf<sf::Event::MouseButtonPressed>();
                if (mb && mb->button == sf::Mouse::Button::Left) {
                    sf::Vector2i pix = mb->position;
                    sf::Vector2f mouseMm((pix.x - WINDOW_MARGIN_PX) / PX_PER_MM, (pix.y - WINDOW_MARGIN_PX) / PX_PER_MM);

                    // check robot
                    float r_px = mmToPx(robot.diameterMm) / 2.0f;
                    sf::Vector2f robotPx(mmToPx(robot.pos.x) + WINDOW_MARGIN_PX, mmToPx(robot.pos.y) + WINDOW_MARGIN_PX);
                    float rdx = float(pix.x) - robotPx.x;
                    float rdy = float(pix.y) - robotPx.y;
                    if (std::sqrt(rdx*rdx + rdy*rdy) <= r_px) {
                        dragging = true;
                        dragOffsetMm = robot.pos - mouseMm; // preserve relative offset
                        continue;
                    }

                    // check ball
                    float b_px = mmToPx(BALL_DIAMETER_MM) / 2.0f;
                    sf::Vector2f ballPx(mmToPx(ballPosMm.x) + WINDOW_MARGIN_PX, mmToPx(ballPosMm.y) + WINDOW_MARGIN_PX);
                    float bdx = float(pix.x) - ballPx.x;
                    float bdy = float(pix.y) - ballPx.y;
                    if (std::sqrt(bdx*bdx + bdy*bdy) <= b_px) {
                        draggingBall = true;
                        ballDragOffsetMm = ballPosMm - mouseMm;
                        ballVelMmS = sf::Vector2f(0.0f, 0.0f); // release a still ball
                        ballHeld = false; // user grabbed the ball off the dribbler
                            continue;
                        }

                        // HUD click area
                        float hudX_click = WINDOW_MARGIN_PX + mmToPx(FIELD_WIDTH_MM) + WINDOW_MARGIN_PX;
                        float hudY_click = WINDOW_MARGIN_PX;
                        const float pad_click = 10.0f;
                        const float btnSize = 28.0f;

                        // Scale control positions (match drawing below)
                        float scaleRowY = hudY_click + pad_click + 40.0f;
                        float moveInputX = hudX_click + pad_click + 160.0f; // x of input box
                        float moveInputW = 80.0f;
                        float moveInputH = 28.0f;

                        float rotInputX = hudX_click + pad_click + 160.0f;
                        float rotRowY = scaleRowY + 40.0f;
                        float rotInputW = 80.0f;
                        float rotInputH = 28.0f;

                        // Click on move input box
                        if (pix.x >= static_cast<int>(std::round(moveInputX)) && pix.x <= static_cast<int>(std::round(moveInputX + moveInputW)) && pix.y >= static_cast<int>(std::round(scaleRowY)) && pix.y <= static_cast<int>(std::round(scaleRowY + moveInputH))) {
                            editMove = true;
                            editRot = false;
                            // store current string
                            moveSpeedStr = std::to_string(moveSpeedScale);
                            continue;
                        }

                        // Click on rot input box
                        if (pix.x >= static_cast<int>(std::round(rotInputX)) && pix.x <= static_cast<int>(std::round(rotInputX + rotInputW)) && pix.y >= static_cast<int>(std::round(rotRowY)) && pix.y <= static_cast<int>(std::round(rotRowY + rotInputH))) {
                            editRot = true;
                            editMove = false;
                            rotSpeedStr = std::to_string(rotSpeedScale);
                            continue;
                        }

                        // check HUD enable/disable buttons (top-left of HUD panel)
                        const float ex = hudX_click + pad_click;
                        const float ey = hudY_click + pad_click;
                        const float dx = hudX_click + pad_click + 36.0f;
                        const float dy = ey;
                        if (pix.x >= static_cast<int>(std::round(ex)) && pix.x <= static_cast<int>(std::round(ex + btnSize)) && pix.y >= static_cast<int>(std::round(ey)) && pix.y <= static_cast<int>(std::round(ey + btnSize))) {
                            robotEnabled = true;
                            robotCurrentlyRunning = true;
                            continue;
                        }
                        if (pix.x >= static_cast<int>(std::round(dx)) && pix.x <= static_cast<int>(std::round(dx + btnSize)) && pix.y >= static_cast<int>(std::round(dy)) && pix.y <= static_cast<int>(std::round(dy + btnSize))) {
                            robotEnabled = false;
                            robotCurrentlyRunning = false;
                            continue;
                        }
                    }
                }

            if (ev.is<sf::Event::MouseButtonReleased>()) {
                const auto *mb = ev.getIf<sf::Event::MouseButtonReleased>();
                if (mb && mb->button == sf::Mouse::Button::Left) {
                    dragging = false; // release control back to robot code
                    draggingBall = false;
                    ballVelMmS = sf::Vector2f(0.0f, 0.0f); // dropped ball must not roll off
                }
            }

            if (ev.is<sf::Event::MouseMoved>()) {
                const auto *mmv = ev.getIf<sf::Event::MouseMoved>();
                if (mmv && dragging) {
                    sf::Vector2i pix = mmv->position;
                    sf::Vector2f mouseMm((pix.x - WINDOW_MARGIN_PX) / PX_PER_MM, (pix.y - WINDOW_MARGIN_PX) / PX_PER_MM);
                    moveWithGoalCollision(robot.pos, mouseMm + dragOffsetMm, robot.diameterMm / 2.0f);
                }
                if (mmv && draggingBall) {
                    sf::Vector2i pix = mmv->position;
                    sf::Vector2f mouseMm((pix.x - WINDOW_MARGIN_PX) / PX_PER_MM, (pix.y - WINDOW_MARGIN_PX) / PX_PER_MM);
                    moveBallWithCollision(ballPosMm, mouseMm + ballDragOffsetMm);
                }
            }
        }

        float dt = clock.restart().asSeconds();

        // Advance simulator time (fixed-step to keep deterministic behavior)
        simMs += SIM_MS_PER_FRAME;
        sim_set_millis(simMs);

        // Clear and draw
        window.clear(sf::Color(60,60,60)); // background outside field

        // Recompute shapes that depend on scaling (if user zoomed)
        greenBg.setSize(sf::Vector2f(mmToPx(FIELD_WIDTH_MM), mmToPx(FIELD_HEIGHT_MM)));
        topLine.setSize(sf::Vector2f(mmToPx(FIELD_WIDTH_MM) - 2*mmToPx(WHITE_LINE_INSET_FROM_WALL_MM), mmToPx(WHITE_LINE_THICKNESS_MM)));
        topLine.setPosition(sf::Vector2f(WINDOW_MARGIN_PX + mmToPx(WHITE_LINE_INSET_FROM_WALL_MM), WINDOW_MARGIN_PX + mmToPx(WHITE_LINE_INSET_FROM_WALL_MM) - mmToPx(WHITE_LINE_THICKNESS_MM)/2.0f));
        bottomLine.setSize(topLine.getSize());
        bottomLine.setPosition(sf::Vector2f(WINDOW_MARGIN_PX + mmToPx(WHITE_LINE_INSET_FROM_WALL_MM), WINDOW_MARGIN_PX + mmToPx(FIELD_HEIGHT_MM) - mmToPx(WHITE_LINE_INSET_FROM_WALL_MM) - mmToPx(WHITE_LINE_THICKNESS_MM)/2.0f));
        leftLine.setSize(sf::Vector2f(mmToPx(WHITE_LINE_THICKNESS_MM), mmToPx(FIELD_HEIGHT_MM) - 2*mmToPx(WHITE_LINE_INSET_FROM_WALL_MM)));
        leftLine.setPosition(sf::Vector2f(WINDOW_MARGIN_PX + mmToPx(WHITE_LINE_INSET_FROM_WALL_MM) - mmToPx(WHITE_LINE_THICKNESS_MM)/2.0f, WINDOW_MARGIN_PX + mmToPx(WHITE_LINE_INSET_FROM_WALL_MM)));
        rightLine.setSize(leftLine.getSize());
        rightLine.setPosition(sf::Vector2f(WINDOW_MARGIN_PX + mmToPx(FIELD_WIDTH_MM) - mmToPx(WHITE_LINE_INSET_FROM_WALL_MM) - mmToPx(WHITE_LINE_THICKNESS_MM)/2.0f, WINDOW_MARGIN_PX + mmToPx(WHITE_LINE_INSET_FROM_WALL_MM)));

        // goals update
        goalW_px = mmToPx(GOAL_WIDTH_MM);
        goalD_px = mmToPx(GOAL_DEPTH_MM);
        centerX_px = WINDOW_MARGIN_PX + mmToPx(FIELD_WIDTH_MM)/2.0f;
        float whiteTopY = WINDOW_MARGIN_PX + mmToPx(WHITE_LINE_INSET_FROM_WALL_MM) + (mmToPx(WHITE_LINE_THICKNESS_MM)/2.0f);
        float topGoalInnerY = whiteTopY;
        topGoal.setSize(sf::Vector2f(goalW_px, goalD_px));
        topGoal.setOrigin(sf::Vector2f(goalW_px/2.0f, goalD_px));
        topGoal.setPosition(sf::Vector2f(centerX_px, topGoalInnerY));
        float whiteBottomY = WINDOW_MARGIN_PX + mmToPx(FIELD_HEIGHT_MM) - mmToPx(WHITE_LINE_INSET_FROM_WALL_MM) - (mmToPx(WHITE_LINE_THICKNESS_MM)/2.0f);
        float bottomGoalInnerY = whiteBottomY;
        bottomGoal.setSize(sf::Vector2f(goalW_px, goalD_px));
        bottomGoal.setOrigin(sf::Vector2f(goalW_px/2.0f, 0.0f));
        bottomGoal.setPosition(sf::Vector2f(centerX_px, bottomGoalInnerY));

        // draw field
        window.draw(greenBg);
        window.draw(topLine);
        window.draw(bottomLine);
        window.draw(leftLine);
        window.draw(rightLine);

        // goals
        window.draw(topGoal);
        window.draw(bottomGoal);

        // Apply MoveProfile to simulated robot physics (if robot code is running and profile active)
        float dt_s = float(SIM_MS_PER_FRAME) / 1000.0f;
        const sf::Vector2f prevRobotPos = robot.pos;
        if (robotCurrentlyRunning && currentMoveProfile.active) {
            // linear speed (mm/s)
            float linear_mm_s = currentMoveProfile.speed * SIM_MAX_LINEAR_MM_S * moveSpeedScale;
            // rotation deg/s
            float rot_deg_s = currentMoveProfile.rotationSpeed * SIM_MAX_ROT_DEG_S * rotSpeedScale;
            // update heading
            // Interpret rotationSpeed as: positive = counterclockwise. Convert to simulator heading which
            // previously treated increasing heading as clockwise, so subtract to make positive -> CCW.
            robot.headingDeg -= rot_deg_s * dt_s;
            // compute world angle for movement: robot.headingDeg + movementDirectionDeg
            float worldDeg = robot.headingDeg + ( -currentMoveProfile.movementDirectionDeg );
            float angleRad = (worldDeg - 90.0f) * 3.14159265f / 180.0f;
            float dx = std::cos(angleRad) * linear_mm_s * dt_s;
            float dy = std::sin(angleRad) * linear_mm_s * dt_s;
            moveWithGoalCollision(robot.pos, robot.pos + sf::Vector2f(dx, dy), robot.diameterMm / 2.0f);
        }

        // Ball physics: roll with friction, bounce off walls/goals, and be
        // pushed out (and given a nudge) whenever it touches the robot.
        // If the dribbler is requested (dribblerShouldRun) and the ball touches
        // the frontal dribbler bar, the ball is held against the bar instead of
        // rolling/bouncing, so it moves and rotates with the robot.
        const sf::Vector2f robotVelMmS = dt_s > 0.0f ? (robot.pos - prevRobotPos) / dt_s : sf::Vector2f(0.0f, 0.0f);
        if (!draggingBall) {
            ballHeld = updateBallDribbling(ballPosMm, ballVelMmS, ballHeld, robot.pos, robot.diameterMm,
                                           robot.headingDeg, robotVelMmS, dribblerShouldRun);
        }
        // The user fully controls the ball while dragging it, so the robot must
        // not shove it around (or give it velocity) mid-drag.
        if (!draggingBall && !ballHeld) {
            moveBall(ballPosMm, ballVelMmS, dt_s);
        }
        if (!draggingBall && !ballHeld) {
            resolveRobotBallCollision(ballPosMm, ballVelMmS, robot.pos, robot.diameterMm / 2.0f, robotVelMmS);
        }
        if (!draggingBall) {
            containBallInGoalBoxes(ballPosMm);
        }
        clampBallToField(ballPosMm);


        // draw robot
        robot.draw(window);

        // draw ball
        float ball_r_px = mmToPx(BALL_DIAMETER_MM) / 2.0f;
        sf::CircleShape ballShape(ball_r_px);
        ballShape.setOrigin(sf::Vector2f(ball_r_px, ball_r_px));
        ballShape.setPosition(sf::Vector2f(mmToPx(ballPosMm.x) + WINDOW_MARGIN_PX, mmToPx(ballPosMm.y) + WINDOW_MARGIN_PX));
        ballShape.setFillColor(sf::Color(255, 165, 0)); // orange
        ballShape.setOutlineColor(sf::Color::Black);
        ballShape.setOutlineThickness(1.0f);
        window.draw(ballShape);
 
        // Update BallPacket based on ball->robot geometry
        // Compute vector from robot to ball in mm
        float dx = ballPosMm.x - robot.pos.x;
        float dy = ballPosMm.y - robot.pos.y;
        float distMm = std::sqrt(dx*dx + dy*dy);
        // Robot front direction (world frame) as unit vector
        float angleRad = (robot.headingDeg - 90.0f) * 3.14159265f / 180.0f;
        sf::Vector2f frontVec(std::cos(angleRad), std::sin(angleRad));
        // vector to ball
        sf::Vector2f v(dx, dy);
        // compute signed angle between frontVec and v
        float dot = frontVec.x * v.x + frontVec.y * v.y;
        float cross = frontVec.x * v.y - frontVec.y * v.x;
        float bearingRad = std::atan2(cross, dot);
        float bearingDeg = bearingRad * 180.0f / 3.14159265f;

        latestBallPacket.detected = true;
        latestBallPacket.angleDeg = bearingDeg; // angle relative to robot front
        latestBallPacket.distanceCM = distMm / 10.0f;
        latestBallPacket.sizeByte = static_cast<uint8_t>(std::min(255, static_cast<int>(BALL_DIAMETER_MM)));
        // Use simulator-driven milliseconds (simMs) for the ball packet timestamp so
        // the robot firmware (which reads millis()) sees a consistent epoch.
        lastBallPacketMs = simMs;

        // HUD panel area on the right side
        float hudX = WINDOW_MARGIN_PX + mmToPx(FIELD_WIDTH_MM) + WINDOW_MARGIN_PX;
        float hudY = WINDOW_MARGIN_PX;
        float hudW = static_cast<float>(HUD_PANEL_WIDTH_PX);
        float hudH = mmToPx(FIELD_HEIGHT_MM);

        // draw HUD background panel
        sf::RectangleShape hudPanel(sf::Vector2f(hudW, hudH));
        hudPanel.setPosition(sf::Vector2f(hudX, hudY));
        hudPanel.setFillColor(sf::Color(30,30,30));
        hudPanel.setOutlineColor(sf::Color::Black);
        hudPanel.setOutlineThickness(2.0f);
        window.draw(hudPanel);

        // Ensure HUD font is available
        static sf::Font hudFont;
        static bool hudFontLoaded = false;
        if (!hudFontLoaded) {
           // Try common Windows fonts as a fallback so HUD works without bundling a font file
           hudFontLoaded = hudFont.openFromFile("C:\\Windows\\Fonts\\arial.ttf")
               || hudFont.openFromFile("C:\\Windows\\Fonts\\segoeui.ttf")
               || hudFont.openFromFile("C:\\Windows\\Fonts\\tahoma.ttf");
        }

        // draw enable/disable buttons at top-left of HUD
        const float pad = 10.0f;
        sf::RectangleShape btnEnable(sf::Vector2f(28.0f, 28.0f));
        btnEnable.setPosition(sf::Vector2f(hudX + pad, hudY + pad));
        btnEnable.setFillColor(robotEnabled ? sf::Color(50,200,50) : sf::Color(80,80,80));
        btnEnable.setOutlineThickness(1.0f);
        btnEnable.setOutlineColor(sf::Color::Black);
        window.draw(btnEnable);
 
        sf::RectangleShape btnDisable(sf::Vector2f(28.0f, 28.0f));
        btnDisable.setPosition(sf::Vector2f(hudX + pad + 36.0f, hudY + pad));
        btnDisable.setFillColor(!robotEnabled ? sf::Color(220,50,50) : sf::Color(80,80,80));
        btnDisable.setOutlineThickness(1.0f);
        btnDisable.setOutlineColor(sf::Color::Black);
        window.draw(btnDisable);

        // Draw labels for robot state
        if (hudFontLoaded) {
            sf::Text stateLabel(hudFont, robotCurrentlyRunning ? "Robot: ENABLED" : "Robot: DISABLED", 14);
            stateLabel.setFillColor(robotCurrentlyRunning ? sf::Color(180,255,180) : sf::Color(200,120,120));
            stateLabel.setPosition(sf::Vector2f(hudX + pad + 72.0f, hudY + pad + 4.0f));
            window.draw(stateLabel);
        }

        // Draw MoveProfile scaling controls
        if (hudFontLoaded) {
            float scaleBaseY = hudY + pad + 40.0f;
            unsigned int fs = 14;
            // Move scale label and input box
            sf::Text moveLabel(hudFont, std::string("Move scale:"), fs);
            moveLabel.setFillColor(sf::Color::White);
            moveLabel.setPosition(sf::Vector2f(hudX + pad, scaleBaseY));
            window.draw(moveLabel);
            // input box for move scale
            float moveInputX = hudX + pad + 160.0f;
            float moveInputY = scaleBaseY;
            float moveInputW = 80.0f;
            float moveInputH = 28.0f;
            sf::RectangleShape moveInputBox(sf::Vector2f(moveInputW, moveInputH));
            moveInputBox.setPosition(sf::Vector2f(moveInputX, moveInputY));
            moveInputBox.setFillColor(sf::Color(220,220,220));
            moveInputBox.setOutlineColor(editMove ? sf::Color::Yellow : sf::Color::Black);
            moveInputBox.setOutlineThickness(1.0f);
            window.draw(moveInputBox);
            std::string showMove = editMove ? moveSpeedStr : std::to_string(moveSpeedScale);
            sf::Text moveText(hudFont, showMove, fs);
            moveText.setFillColor(sf::Color::Black);
            moveText.setPosition(sf::Vector2f(moveInputX + 6.0f, moveInputY));
            window.draw(moveText);

            // Rotation scale label and input box
            float rotY = scaleBaseY + 40.0f;
            sf::Text rotLabel(hudFont, std::string("Rot scale:"), fs);
            rotLabel.setFillColor(sf::Color::White);
            rotLabel.setPosition(sf::Vector2f(hudX + pad, rotY));
            window.draw(rotLabel);
            float rotInputX = hudX + pad + 160.0f;
            float rotInputY = rotY;
            float rotInputW = 80.0f;
            float rotInputH = 28.0f;
            sf::RectangleShape rotInputBox(sf::Vector2f(rotInputW, rotInputH));
            rotInputBox.setPosition(sf::Vector2f(rotInputX, rotInputY));
            rotInputBox.setFillColor(sf::Color(220,220,220));
            rotInputBox.setOutlineColor(editRot ? sf::Color::Yellow : sf::Color::Black);
            rotInputBox.setOutlineThickness(1.0f);
            window.draw(rotInputBox);
            std::string showRot = editRot ? rotSpeedStr : std::to_string(rotSpeedScale);
            sf::Text rotText(hudFont, showRot, fs);
            rotText.setFillColor(sf::Color::Black);
            rotText.setPosition(sf::Vector2f(rotInputX + 6.0f, rotInputY));
            window.draw(rotText);

        }

        // HUD: render BallPacket and MoveProfile inside the panel

        if (hudFontLoaded) {
            std::string bp1 = std::string("BallPacket.detected: ") + (latestBallPacket.detected ? "true" : "false");
            std::string bp2 = "angleDeg: " + std::to_string(latestBallPacket.angleDeg);
            std::string bp3 = "distanceCM: " + std::to_string(latestBallPacket.distanceCM);
            std::string bp4 = "size: " + std::to_string((int)latestBallPacket.sizeByte);

            std::string mp1 = std::string("MoveProfile.active: ") + (currentMoveProfile.active ? "true" : "false");
            std::string mp2 = "dirDeg: " + std::to_string(currentMoveProfile.movementDirectionDeg);
            std::string mp3 = "speed: " + std::to_string(currentMoveProfile.speed);
            std::string mp4 = "rot: " + std::to_string(currentMoveProfile.rotationSpeed);

            std::string dr1 = std::string("Dribbler: ") + (dribblerShouldRun ? "RUNNING" : "off");
            std::string dr2 = std::string("Ball held: ") + (ballHeld ? "YES" : "no");

            const float pad = 10.0f;
            float tx = hudX + pad;
                        const float HUD_CONTROLS_HEIGHT = 120.0f; // space reserved for enable buttons and scale controls
                        float ty = hudY + pad + HUD_CONTROLS_HEIGHT; // start text below controls
            unsigned int fs = 16;

            sf::Text t1(hudFont, bp1, fs); t1.setFillColor(sf::Color::White); t1.setPosition(sf::Vector2f(tx, ty)); window.draw(t1); ty += 22.0f;
            sf::Text t2(hudFont, bp2, fs); t2.setFillColor(sf::Color::White); t2.setPosition(sf::Vector2f(tx, ty)); window.draw(t2); ty += 22.0f;
            sf::Text t3(hudFont, bp3, fs); t3.setFillColor(sf::Color::White); t3.setPosition(sf::Vector2f(tx, ty)); window.draw(t3); ty += 22.0f;
            sf::Text t4(hudFont, bp4, fs); t4.setFillColor(sf::Color::White); t4.setPosition(sf::Vector2f(tx, ty)); window.draw(t4); ty += 32.0f;

            sf::Text m1(hudFont, mp1, fs); m1.setFillColor(sf::Color::White); m1.setPosition(sf::Vector2f(tx, ty)); window.draw(m1); ty += 22.0f;
            sf::Text m2(hudFont, mp2, fs); m2.setFillColor(sf::Color::White); m2.setPosition(sf::Vector2f(tx, ty)); window.draw(m2); ty += 22.0f;
            sf::Text m3(hudFont, mp3, fs); m3.setFillColor(sf::Color::White); m3.setPosition(sf::Vector2f(tx, ty)); window.draw(m3); ty += 22.0f;
            sf::Text m4(hudFont, mp4, fs); m4.setFillColor(sf::Color::White); m4.setPosition(sf::Vector2f(tx, ty)); ty += 32.0f;

            sf::Text d1(hudFont, dr1, fs); d1.setFillColor(sf::Color(255, 220, 140)); d1.setPosition(sf::Vector2f(tx, ty)); window.draw(d1); ty += 22.0f;
            sf::Text d2(hudFont, dr2, fs); d2.setFillColor(ballHeld ? sf::Color(140, 255, 140) : sf::Color::White); d2.setPosition(sf::Vector2f(tx, ty)); window.draw(d2); ty += 22.0f;
        }

        window.display();
    }

    // Stop robot thread cleanly
    robot_stop();

    return 0;
}
