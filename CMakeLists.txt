cmake_minimum_required(VERSION 3.12)

project(CoverageControl VERSION 0.2 LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)
# set(CMAKE_FIND_DEBUG_MODE 1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

add_subdirectory(${PROJECT_SOURCE_DIR}/core)
