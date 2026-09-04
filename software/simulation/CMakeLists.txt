cmake_minimum_required(VERSION 3.10)
project(rcj_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML COMPONENTS Graphics Window System REQUIRED)

# Collect sources
set(SIM_SOURCES
  src/main.cpp
  src/robot_wrapper.cpp
  src/sim_hal/sim_hal.cpp
  src/sim_stubs/sim_display_stub.cpp
  src/sim_stubs/sim_imu_stub.cpp
  src/sim_stubs/sim_dribbler_stub.cpp
  src/sim_stubs/sim_battery_stub.cpp
  # robot code sources (selected subset from white robot)
  ${CMAKE_SOURCE_DIR}/../white/src/robot.cpp
  ${CMAKE_SOURCE_DIR}/../white/src/cpp/subsystems/vision.cpp
  ${CMAKE_SOURCE_DIR}/../white/src/cpp/subsystems/localisation.cpp
  ${CMAKE_SOURCE_DIR}/../white/src/cpp/subsystems/attack.cpp
  ${CMAKE_SOURCE_DIR}/../white/src/cpp/subsystems/defend.cpp
  ${CMAKE_SOURCE_DIR}/../white/src/cpp/subsystems/drivebase.cpp
  ${CMAKE_SOURCE_DIR}/../white/src/cpp/subsystems/robot_state.cpp
)

add_executable(simulation ${SIM_SOURCES})

# Include search order:
#   1. sim/src         – so "sim_hal/Arduino.h" resolves to src/sim_hal/Arduino.h,
#                        and "sim_hal/Wire.h" works as well
#   2. sim/include     – sim-specific stubs for imu.h, display.h, Adafruit headers
#   3. white/src       – robot.cpp and other source-level #includes (e.g. "include/...")
#   4. white/src/include – robot_config.h, robot_state.h, vision.h, etc.
target_include_directories(simulation PRIVATE
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/../white/src
  ${CMAKE_SOURCE_DIR}/../white/src/include
)

target_link_libraries(simulation PRIVATE SFML::Graphics SFML::Window SFML::System)

# Define Arduino analog pin macros so robot_config.h can use A0..A15 as constants
target_compile_definitions(simulation PRIVATE
  A0=0 A1=1 A2=2 A3=3 A4=4 A5=5 A6=6 A7=7 A8=8 A9=9 A10=10 A11=11 A12=12 A13=13 A14=14 A15=15)
