cmake_minimum_required(VERSION 3.12)

project(CoverageControl VERSION 0.1 LANGUAGES CUDA CXX)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)


find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PythonLibs REQUIRED)

add_library(gpu_library SHARED src/generate_world_map.cu)
target_include_directories(gpu_library PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/extern/cuda_helpers/)
target_link_libraries(gpu_library PRIVATE Eigen3::Eigen)
set_target_properties(gpu_library PROPERTIES CUDA_ARCHITECTURES native)

add_executable(map_generation ${PROJECT_SOURCE_DIR}/test/map_generation.cpp)
target_include_directories(map_generation PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(map_generation PRIVATE Eigen3::Eigen gpu_library)

add_executable(robot_map ${PROJECT_SOURCE_DIR}/test/robot_map.cpp)
target_include_directories(robot_map PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(robot_map PRIVATE Eigen3::Eigen gpu_library)
