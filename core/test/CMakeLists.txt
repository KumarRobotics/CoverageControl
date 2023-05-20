cmake_minimum_required(VERSION 3.15)

project(CoverageControl VERSION 0.3 LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(CGAL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CoverageControl REQUIRED)

###########
## Tests ##
###########

add_executable(system_map system_map.cpp)
target_link_libraries(system_map PUBLIC CoverageControl)
install(TARGETS system_map DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(coverage_algorithm coverage_algorithm.cpp)
target_link_libraries(coverage_algorithm PRIVATE CoverageControl)
install(TARGETS coverage_algorithm DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_executable(geo_transforms ${PROJECT_SOURCE_DIR}/test/geo_transforms.cpp)
# target_include_directories(geo_transforms PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/extern)
# target_link_libraries(geo_transforms PRIVATE Eigen3::Eigen CoverageControl yaml-cpp OpenMP::OpenMP_CXX ${GeographicLib_LIBRARIES})
# install(TARGETS geo_transforms DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(map_generation map_generation.cpp)
target_link_libraries(map_generation PRIVATE CoverageControl)
install(TARGETS map_generation DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_executable(robot_map ${PROJECT_SOURCE_DIR}/test/robot_map.cpp)
# target_include_directories(robot_map PRIVATE ${PROJECT_SOURCE_DIR}/include)
# target_link_libraries(robot_map PRIVATE Eigen3::Eigen CoverageControl)
# install(TARGETS robot_map DESTINATION ${CMAKE_INSTALL_BINDIR})
