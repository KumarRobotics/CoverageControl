cmake_minimum_required(VERSION 3.5.1)
project(coverage_control_lib)

project(coverage_control_lib VERSION 0.3 LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)
# set(CMAKE_FIND_DEBUG_MODE 1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

set(PKG_DEPENDS rospy)

set(BUILD_DEPENDS ${PKG_DEPENDS})

set(RUN_DEPENDS ${PKG_DEPENDS})

add_subdirectory(${PROJECT_SOURCE_DIR}/core)
#######################################
## Find dependencies
#######################################
find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPENDS})

include_directories(
  include
  )

catkin_package(
  CATKIN_DEPENDS ${RUN_DEPENDS}
)

execute_process(COMMAND bash ${PROJECT_SOURCE_DIR}/install/install_dependencies.sh ${PROJECT_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/../ ${CMAKE_INSTALL_PREFIX})

add_subdirectory(${PROJECT_SOURCE_DIR}/cppsrc/core)
