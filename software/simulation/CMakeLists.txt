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
  # robot code sources (selected subset)
  ${CMAKE_SOURCE_DIR}/../code/src/robot.cpp
  ${CMAKE_SOURCE_DIR}/../code/src/cpp/subsystems/vision.cpp
  ${CMAKE_SOURCE_DIR}/../code/src/cpp/subsystems/drivebase.cpp
  ${CMAKE_SOURCE_DIR}/../code/src/cpp/subsystems/robot_state.cpp
)

add_executable(simulation ${SIM_SOURCES})

# include local headers and the robot code headers so the sim can read/write robot_state
# ensure sim/src is searched before the robot include dir so our Arduino.h shim is used
target_include_directories(simulation PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/sim_hal include ${CMAKE_SOURCE_DIR}/../code/src ${CMAKE_SOURCE_DIR}/../code/src/include)

target_link_libraries(simulation PRIVATE SFML::Graphics SFML::Window SFML::System)

# Define Arduino analog pin macros so robot_config.h can use A0..A15 as constants
target_compile_definitions(simulation PRIVATE
  A0=0 A1=1 A2=2 A3=3 A4=4 A5=5 A6=6 A7=7 A8=8 A9=9 A10=10 A11=11 A12=12 A13=13 A14=14 A15=15)

