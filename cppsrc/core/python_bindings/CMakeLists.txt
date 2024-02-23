cmake_minimum_required(VERSION 3.16)
project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS True)

set(PYBIND11_NEWPYTHON True)
find_package(pybind11 CONFIG REQUIRED)

find_package(CoverageControl REQUIRED)

pybind11_add_module(CoverageControl MODULE src/coverage_control_python_binds.cpp)
target_link_libraries(CoverageControl PRIVATE CoverageControl::CoverageControl)
install(TARGETS CoverageControl LIBRARY DESTINATION .)

