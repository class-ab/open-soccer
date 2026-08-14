#include <SFML/Graphics.hpp>
#include <cmath>
#include <string>
#include <algorithm>
#include <iostream>

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

// Rendering
float PX_PER_MM = 0.35f; // default scale (pixels per millimetre). Adjust for window size.
const int WINDOW_MARGIN_PX = 40;

// Helper conversions
inline float mmToPx(float mm) { return mm * PX_PER_MM; }

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
    float max_px_per_mm_w = float((int)screenW - 2*WINDOW_MARGIN_PX - EXTRA_MARGIN_W) / FIELD_WIDTH_MM;
    float max_px_per_mm_h = float((int)screenH - 2*WINDOW_MARGIN_PX - EXTRA_MARGIN_H) / FIELD_HEIGHT_MM;
    float chosen = std::min(max_px_per_mm_w, max_px_per_mm_h);
    // clamp to reasonable range
    PX_PER_MM = std::max(0.05f, std::min(chosen, 5.0f));

    // Compute window size in pixels from field dims using chosen scale
    int winW = static_cast<int>(std::round(mmToPx(FIELD_WIDTH_MM))) + 2*WINDOW_MARGIN_PX;
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

    // Top goal (drawn just above the top white line; inner edge positioned GOAL_DEPTH_MM from inner edge of white line)
    // Compute inner edge of the white top line y position (in pixels)
    float whiteTopInnerY_px = WINDOW_MARGIN_PX + inset_px + (lineThickness_px/2.0f);
    // Place goal so that its inner edge is GOAL_DEPTH_MM from that inner edge
    float topGoalInnerY_px = whiteTopInnerY_px + mmToPx(GOAL_DEPTH_MM);
    sf::RectangleShape topGoal(sf::Vector2f(goalW_px, goalD_px));
    topGoal.setOrigin(sf::Vector2f(goalW_px/2.0f, goalD_px)); // origin at bottom-center of goal rect
    topGoal.setPosition(sf::Vector2f(centerX_px, topGoalInnerY_px));
    topGoal.setFillColor(sf::Color(230,230,230));
    topGoal.setOutlineThickness(2.0f);
    topGoal.setOutlineColor(sf::Color::Black);

    // Bottom goal (symmetrical)
    float whiteBottomInnerY_px = WINDOW_MARGIN_PX + outerH_px - inset_px - (lineThickness_px/2.0f);
    float bottomGoalInnerY_px = whiteBottomInnerY_px - mmToPx(GOAL_DEPTH_MM);
    sf::RectangleShape bottomGoal(sf::Vector2f(goalW_px, goalD_px));
    bottomGoal.setOrigin(sf::Vector2f(goalW_px/2.0f, 0.0f)); // origin at top-center
    bottomGoal.setPosition(sf::Vector2f(centerX_px, bottomGoalInnerY_px));
    bottomGoal.setFillColor(sf::Color(230,230,230));
    bottomGoal.setOutlineThickness(2.0f);
    bottomGoal.setOutlineColor(sf::Color::Black);

    // Basic control variables
    const float ROBOT_SPEED_MM_S = 220.0f; // forward speed mm per second
    const float ROTATION_SPEED_DEG_S = 120.0f;

    sf::Clock clock;

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
                }
            }
        }

        float dt = clock.restart().asSeconds();

        // Keyboard movement: Up/Down forward/back, Left/Right rotate
        bool up = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up);
        bool down = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down);
        bool left = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left);
        bool right = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right);

        if (left) robot.headingDeg -= ROTATION_SPEED_DEG_S * dt;
        if (right) robot.headingDeg += ROTATION_SPEED_DEG_S * dt;

        float moveDir = 0.0f;
        if (up) moveDir += 1.0f;
        if (down) moveDir -= 1.0f;

        if (moveDir != 0.0f) {
            float angleRad = (robot.headingDeg - 90.0f) * 3.14159265f / 180.0f;
            sf::Vector2f dir(std::cos(angleRad), std::sin(angleRad));
            robot.pos += dir * (ROBOT_SPEED_MM_S * dt * moveDir);
            // clamp to field bounds (keep center of robot within outer green area)
            float halfRobot = robot.diameterMm/2.0f;
            robot.pos.x = std::max(halfRobot, std::min(FIELD_WIDTH_MM - halfRobot, robot.pos.x));
            robot.pos.y = std::max(halfRobot, std::min(FIELD_HEIGHT_MM - halfRobot, robot.pos.y));
        }

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
        float topGoalInnerY = whiteTopY + mmToPx(GOAL_DEPTH_MM);
        topGoal.setSize(sf::Vector2f(goalW_px, goalD_px));
        topGoal.setOrigin(sf::Vector2f(goalW_px/2.0f, goalD_px));
        topGoal.setPosition(sf::Vector2f(centerX_px, topGoalInnerY));
        float whiteBottomY = WINDOW_MARGIN_PX + mmToPx(FIELD_HEIGHT_MM) - mmToPx(WHITE_LINE_INSET_FROM_WALL_MM) - (mmToPx(WHITE_LINE_THICKNESS_MM)/2.0f);
        float bottomGoalInnerY = whiteBottomY - mmToPx(GOAL_DEPTH_MM);
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

        // draw robot
        robot.draw(window);

        // HUD text
        sf::Font font;
        static bool fontLoaded = false;
        if (!fontLoaded) {
            // use default SFML font fallback if available; otherwise skip
            // (embedding a font is outside scope)
            fontLoaded = true;
        }

        window.display();
    }

    return 0;
}
