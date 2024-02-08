cmake_minimum_required(VERSION 3.10)

project(kinematics)

set(CMAKE_BUILD_TYPE Debug)
include_directories(include)

add_library(kinematics STATIC src/kinematics.cpp src/utils.cpp)

add_custom_target(kinematics_directory ALL COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_HOME_DIRECTORY}/library/lib)

if ( MSVC )
    set_target_properties(kinematics PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_HOME_DIRECTORY}/library/lib)
    set_target_properties(kinematics PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_HOME_DIRECTORY}/library/lib)
else()
    set_target_properties(kinematics PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/library/lib)
endif ()

add_executable(main src/main.cpp)

target_link_libraries(main kinematics)