cmake_minimum_required(VERSION 3.15)

project(CoverageControl VERSION 0.3 LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(CGAL REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(CoverageControl REQUIRED)

pybind11_add_module(pyCoverageControl MODULE src/coverage_control_python_binds.cpp)
target_include_directories(pyCoverageControl PUBLIC ${COVERAGECONTROL_INCLUDE_DIRS})
target_link_libraries(pyCoverageControl PUBLIC CoverageControl)

