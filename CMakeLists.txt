cmake_minimum_required(VERSION 3.12)

project(CoverageControl VERSION 0.3 LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

add_subdirectory(${PROJECT_SOURCE_DIR}/core)


find_package(Torch REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS} -Wall -pedantic")
add_executable(example-app example-app.cpp)
target_link_libraries(example-app ${TORCH_LIBRARIES} CoverageControl)
set_property(TARGET example-app PROPERTY CXX_STANDARD 14)
