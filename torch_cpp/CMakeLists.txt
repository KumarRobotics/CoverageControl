cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(CoverageControlTorch VERSION 0.4 LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)

find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(CGAL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CoverageControl REQUIRED)

add_executable(example-app src/test.cpp)
target_include_directories(example-app PRIVATE ${CoverageControl_INCLUDE_DIR})
target_link_libraries(example-app PRIVATE ${TORCH_LIBRARIES} CoverageControl)
