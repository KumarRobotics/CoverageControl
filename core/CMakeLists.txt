cmake_minimum_required(VERSION 3.12)

project(pyCoverageControl VERSION 0.2 LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)
# set(CMAKE_FIND_DEBUG_MODE 1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

list(APPEND CMAKE_PREFIX_PATH $ENV{COVERAGECONTROL_WS}/install)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(CGAL REQUIRED)
find_package(GeographicLib REQUIRED)

#############################
## CoverageControl library ##
#############################

add_library(CoverageControl STATIC src/generate_world_map.cu src/polygon_utils.cpp src/parameters.cpp src/voronoi.cpp ${PROJECT_SOURCE_DIR}/extern/lsap/Hungarian.cpp)
target_include_directories(CoverageControl PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/extern)
target_link_libraries(CoverageControl PRIVATE Eigen3::Eigen m stdc++fs yaml-cpp OpenMP::OpenMP_CXX CGAL::CGAL pthread ${GeographicLib_LIBRARIES})
set_target_properties(CoverageControl PROPERTIES CUDA_ARCHITECTURES native POSITION_INDEPENDENT_CODE ON)

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

##################################
## CoverageControl python binds ##
##################################
pybind11_add_module(pyCoverageControl MODULE src/coverage_control_python_binds.cpp)
target_include_directories(pyCoverageControl PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/extern ${pybind11_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
target_link_libraries(pyCoverageControl PRIVATE CoverageControl ${PYTHON_LIBRARIES} Eigen3::Eigen m yaml-cpp OpenMP::OpenMP_CXX CGAL::CGAL pthread ${GeographicLib_LIBRARIES})
target_link_directories(pyCoverageControl PRIVATE ${CMAKE_INSTALL_LIBDIR})

###########
## Tests ##
###########

add_executable(map_generation ${PROJECT_SOURCE_DIR}/test/map_generation.cpp)
target_include_directories(map_generation PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/extern)
target_link_libraries(map_generation PRIVATE Eigen3::Eigen CoverageControl yaml-cpp OpenMP::OpenMP_CXX)
install(TARGETS map_generation DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(oracle ${PROJECT_SOURCE_DIR}/test/oracle.cpp)
target_include_directories(oracle PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/extern)
target_link_libraries(oracle PRIVATE Eigen3::Eigen CoverageControl yaml-cpp OpenMP::OpenMP_CXX)
install(TARGETS oracle DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(geo_transforms ${PROJECT_SOURCE_DIR}/test/geo_transforms.cpp)
target_include_directories(geo_transforms PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/extern)
target_link_libraries(geo_transforms PRIVATE Eigen3::Eigen CoverageControl yaml-cpp OpenMP::OpenMP_CXX ${GeographicLib_LIBRARIES})
install(TARGETS geo_transforms DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_executable(map_generation ${PROJECT_SOURCE_DIR}/test/map_generation.cpp)
# target_include_directories(map_generation PRIVATE ${PROJECT_SOURCE_DIR}/include)
# target_link_libraries(map_generation PRIVATE Eigen3::Eigen CoverageControl)
# install(TARGETS map_generation DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_executable(robot_map ${PROJECT_SOURCE_DIR}/test/robot_map.cpp)
# target_include_directories(robot_map PRIVATE ${PROJECT_SOURCE_DIR}/include)
# target_link_libraries(robot_map PRIVATE Eigen3::Eigen CoverageControl)
# install(TARGETS robot_map DESTINATION ${CMAKE_INSTALL_BINDIR})
