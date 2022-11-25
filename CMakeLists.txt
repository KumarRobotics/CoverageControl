cmake_minimum_required(VERSION 3.12)

project(CoverageControl VERSION 0.1 LANGUAGES CUDA CXX)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Python COMPONENTS Interpreter Development)
add_subdirectory(${PROJECT_SOURCE_DIR}/extern/pybind11)
find_package(pybind11 CONFIG)

#############################
## CoverageControl library ##
#############################

add_library(CoverageControl SHARED src/generate_world_map.cu)
target_include_directories(CoverageControl PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/extern/cuda_helpers/)
target_link_libraries(CoverageControl PRIVATE Eigen3::Eigen m stdc++fs)
set_target_properties(CoverageControl PROPERTIES CUDA_ARCHITECTURES native)

install(TARGETS CoverageControl EXPORT CoverageControl-targets
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY
	${PROJECT_SOURCE_DIR}/include/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS CoverageControl LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(EXPORT CoverageControl-targets
	FILE CoverageControl-targets.cmake
	DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/CoverageControl")

configure_file(CoverageControl_config.h.in CoverageControl_config.h)

##################################
## CoverageControl python binds ##
##################################
pybind11_add_module(pyCoverageControl MODULE src/coverage_control_python_binds.cpp)
target_include_directories(pyCoverageControl PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/extern/pybind11/include/ ${PYTHON_INCLUDE_DIRS})
target_link_libraries(pyCoverageControl PRIVATE CoverageControl ${PYTHON_LIBRARIES} Eigen3::Eigen m)
target_link_directories(pyCoverageControl PRIVATE ${CMAKE_INSTALL_LIBDIR})
install(TARGETS pyCoverageControl LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

###########
## Tests ##
###########

add_executable(map_generation ${PROJECT_SOURCE_DIR}/test/map_generation.cpp)
target_include_directories(map_generation PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(map_generation PRIVATE Eigen3::Eigen CoverageControl)
install(TARGETS map_generation DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(robot_map ${PROJECT_SOURCE_DIR}/test/robot_map.cpp)
target_include_directories(robot_map PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(robot_map PRIVATE Eigen3::Eigen CoverageControl)
install(TARGETS robot_map DESTINATION ${CMAKE_INSTALL_BINDIR})
