cmake_minimum_required(VERSION 3.16)
project(robot_telemetry LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Threads REQUIRED)

add_executable(telemetry_sim
    src/main.cpp

    )
target_include_directories(telemetry_sim PRIVATE include)

target_link_libraries(telemetry_sim PRIVATE Threads::Threads)