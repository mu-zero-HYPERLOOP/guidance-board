######################################################################
# @author      : karl (karlsasssie@gmail.com)
# @file        : CMakeLists
# @created     : Friday Apr 07, 2023 15:39:05 CEST
######################################################################

cmake_minimum_required(VERSION 2.8)
project(guidance-board CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME} 
  mock/main.cpp
  mock/can.cpp
  src/canzero/canzero.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE src/)

