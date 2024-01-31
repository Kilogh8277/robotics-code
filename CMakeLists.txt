cmake_minimum_required(VERSION 3.10)

project(kinematics)

set(CMAKE_BUILD_TYPE Debug)
include_directories(include)

add_executable(main src/main.cpp src/transform.cpp src/utils.cpp)