cmake_minimum_required(VERSION 3.15)
project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS True)

set(PYBIND11_NEWPYTHON True)
find_package(pybind11 CONFIG REQUIRED)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(CGAL REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(CoverageControlCore REQUIRED)

pybind11_add_module(pyCoverageControl MODULE src/coverage_control_python_binds.cpp)
target_link_libraries(pyCoverageControl PUBLIC CoverageControlCore)
install(TARGETS pyCoverageControl LIBRARY DESTINATION .)

