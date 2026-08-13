Robocup Junior Open Soccer — 2D Simulation (SFML)

Overview

This folder contains a minimal 2D simulation of an RCJ Open Soccer field and a single robot with a frontal dribbler. The simulation uses SFML for rendering.

Field constants (from user):
- Total green area: 1430 mm (width) x 1820 mm (height)
- White lines: 50 mm thick, positioned 250 mm from the walls on every side (these form the play border)
- Goals: 450 mm wide, centered on short sides, positioned with 74 mm depth from the inner edge of the white line

Notes

- The simulation is purposely 2D and parameterised with constants at the top of src/main.cpp so you can tweak sizes or scale easily.
- Robot size and height are configurable via constants; no official robot max was required for this task.

Build (requires SFML 2.5+ and CMake)

Windows (example using vcpkg or system-installed SFML):
1. mkdir build && cd build
2. cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release ..
3. cmake --build .
4. Run .\simulation.exe

Linux / macOS:
1. mkdir build && cd build
2. cmake ..
3. make
4. ./simulation

If CMake cannot find SFML, install SFML for your platform (or use a package manager such as vcpkg, apt, brew) and ensure SFML is discoverable by CMake.

Usage

- Arrow keys: move forward/back and rotate
- R: reset robot to center
- +/-: adjust drawing scale
- Escape: quit

Files
- src/main.cpp : main simulation code
- CMakeLists.txt : CMake project file

