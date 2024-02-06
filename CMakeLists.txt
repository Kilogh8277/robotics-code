cmake_minimum_required(VERSION 3.10)

project(kinematics)

set(CMAKE_BUILD_TYPE Debug)
include_directories(include)

add_library(kinematics STATIC src/kinematics.cpp src/utils.cpp)

add_executable(main src/main.cpp)

target_link_libraries(main kinematics)