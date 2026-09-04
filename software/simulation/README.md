# RCJ Open Soccer — 2D Simulation (SFML)

A minimal 2D simulation of an RCJ Open Soccer field with a single robot (with a
frontal dribbler) and a ball. It runs the real robot firmware on a background
thread through a simulator HAL, so the robot's `loop()` runs while the render
loop animates the field.

## Field constants

- Total green area: 1430 mm (width) x 1820 mm (height)
- White lines: 50 mm thick, positioned 250 mm from the walls on every side
  (these form the play border)
- Goals: 450 mm wide, centered on the short sides, 74 mm deep from the inner
  edge of the white line
- Robot: 180 mm diameter (configurable) with a 40 mm wide frontal dribbler bar
- Ball: 40 mm diameter

All constants live at the top of `src/main.cpp` and are expressed in
millimetres, so sizes and the rendering scale can be tweaked easily.

## Physics / behaviour

- Ball and robot are clamped to the field walls; goals are solid boxes.
- Dragging/dropping the ball places it exactly where the cursor is (drag
  follows the mouse, clamped only to the field). It can be placed over or
  behind a goal — it no longer snaps to the goal surface.
- Rolling ball friction, wall/goal bounces, and robot-ball contact are
  simulated.
- Dribbling: the sim reads the shared `dribblerShouldRun` flag. When the robot
  requests the dribbler and the ball touches the frontal dribbler bar, the ball
  is held against the bar and follows the robot through translation and
  rotation. It releases when the dribbler is switched off or the user grabs it.
- No scoring yet: the ball is kept out of the goal boxes themselves.

## Build

Requires CMake and SFML. On Windows the project has been built with Visual
Studio 2022 (generated with `cmake -G "Visual Studio 17 2022"`).

```
cmake -S . -B build
cmake --build build --config Release
```

On this machine SFML is installed via vcpkg, so configure with the SFML dir:

```
cmake -S . -B build "-DSFML_DIR=C:\Users\jared\VSCode\vcpkg\installed\x64-windows\share\sfml"
cmake --build build --config Release
```

Then run `build\Release\simulation.exe` (Windows) or `build/simulation`
(Linux/macOS).

> The sim links against a selected subset of the robot code
> (`../white/src/robot.cpp`, `vision.cpp`, `drivebase.cpp`, `robot_state.cpp`)
> and uses `sim_hal/Arduino.h` + the `sim_stubs/` shims so firmware compiles
> unmodified outside Arduino. No robot code is changed to make the sim work.

If CMake cannot find SFML, install it for your platform (vcpkg, apt, brew, …)
and make sure it is discoverable by CMake.

## Usage

Mouse:
- Left-drag on the **robot** to move it (uses goal collision).
- Left-drag on the **ball** to move it (follows the cursor, field-only).
  Releasing places the ball exactly there.

Keyboard:
- `R` — reset robot to field centre
- `Escape` — quit (or cancel an in-progress HUD text edit)

HUD panel (right side):
- **Enable / Disable** buttons toggle the robot firmware run state.
- **Move** and **Rot** fields set the move/rotation speed scale used when
  applying the robot's active `MoveProfile` to simulated motion.
- Readouts: BallPacket (detected / angle / distance), MoveProfile (active /
  direction / speed / rotation), and Dribbler + Ball held state.

## Files

- `src/main.cpp` — main simulation, physics/collision, rendering, input, HUD
- `src/robot_wrapper.cpp` — runs `setup()`/`loop()` of the firmware on a thread
- `src/sim_hal/` — Arduino-style shim (`Arduino.h`) and simulator time control
- `src/sim_stubs/` — stubbed firmware peripherals (display, IMU, dribbler, battery)
- `src/sim_state.cpp` — shared `currentMoveProfile`, `dribblerShouldRun`, etc.
- `CMakeLists.txt` — CMake project
