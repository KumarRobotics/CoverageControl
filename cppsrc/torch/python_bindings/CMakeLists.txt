cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
project(CoverageControlTorch VERSION 0.4 LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS True)

find_package(Torch EXACT REQUIRED PATHS $ENV{Torch_DIR} NO_DEFAULT_PATH)
find_package(TorchVision REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS} -fvisibility=hidden")
message(STATUS "Torch_LIBRARIES: ${TORCH_LIBRARIES}")

set(PYBIND11_NEWPYTHON True)
find_package(pybind11 REQUIRED)

find_package(OpenMP REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(CGAL 5.6 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CoverageControl REQUIRED)
find_package(CoverageControlTorch REQUIRED)

add_library(torch_custom SHARED IMPORTED GLOBAL)
set_target_properties(torch_custom PROPERTIES IMPORTED_LOCATION $ENV{LIBTORCH_PYTHON_PATH})

pybind11_add_module(pyCoverageControlTorch MODULE src/coverage_control_torch_extension.cpp)
target_include_directories(pyCoverageControlTorch PRIVATE ${TORCH_INCLUDE_DIRS})
target_link_libraries(pyCoverageControlTorch PUBLIC torch_custom CoverageControlTorch "${TORCH_LIBRARIES}")
install(TARGETS pyCoverageControlTorch LIBRARY DESTINATION .)

