cmake_minimum_required(VERSION 3.10)
project(rcj_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(simulation src/main.cpp)

target_include_directories(simulation PRIVATE include)
target_link_libraries(simulation PRIVATE sfml-graphics sfml-window sfml-system)
