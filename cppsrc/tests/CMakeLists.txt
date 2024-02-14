cmake_minimum_required(VERSION 3.16)
project(CoverageControlTests VERSION 0.3 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

option(WITH_CUDA "Build with CUDA support" ON)

if(WITH_CUDA)
	cmake_minimum_required(VERSION 3.24)
	enable_language(CUDA)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

find_package(CoverageControlCore REQUIRED)

if(NOT DEFINED WITH_TORCH)
	set(WITH_TORCH OFF)
endif()

###########
## Tests ##
###########

add_executable(system_map system_map.cpp)
target_link_libraries(system_map PRIVATE CoverageControl::CoverageControlCore)
install(TARGETS system_map DESTINATION ${CMAKE_INSTALL_BINDIR})
# set_target_properties(system_map PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}")

if(WITH_TORCH)
	find_package(Torch REQUIRED)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
	get_filename_component(TORCH_LIBRARY_DIR "${TORCH_LIBRARIES}" DIRECTORY)
endif()

if(WITH_TORCH)
	find_package(CoverageControlTorch REQUIRED)
endif()

# add_executable(geo_transforms ${PROJECT_SOURCE_DIR}/test/geo_transforms.cpp)
# target_include_directories(geo_transforms PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/extern)
# target_link_libraries(geo_transforms PRIVATE Eigen3::Eigen CoverageControlTorch OpenMP::OpenMP_CXX ${GeographicLib_LIBRARIES})
# install(TARGETS geo_transforms DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_executable(robot_map ${PROJECT_SOURCE_DIR}/test/robot_map.cpp)
# target_include_directories(robot_map PRIVATE ${PROJECT_SOURCE_DIR}/include)
# target_link_libraries(robot_map PRIVATE Eigen3::Eigen CoverageControlTorch)
# install(TARGETS robot_map DESTINATION ${CMAKE_INSTALL_BINDIR})

if(WITH_TORCH)
	set(dependencies_list CoverageControl::CoverageControlTorch ${TORCH_LIBRARIES})

	add_executable(map_generation map_generation.cpp)
	target_link_libraries(map_generation PRIVATE ${dependencies_list})
	install(TARGETS map_generation DESTINATION ${CMAKE_INSTALL_BINDIR})

	add_executable(torch_test torch_test.cpp)
	target_link_libraries(torch_test PRIVATE ${dependencies_list})
	install(TARGETS torch_test DESTINATION ${CMAKE_INSTALL_BINDIR})

	add_executable(torch_map_conversion torch_map_conversion.cpp)
	target_link_libraries(torch_map_conversion PRIVATE ${dependencies_list})
	install(TARGETS torch_map_conversion DESTINATION ${CMAKE_INSTALL_BINDIR})

	add_executable(test_maps test_maps.cpp)
	target_link_libraries(test_maps PRIVATE ${dependencies_list})
	install(TARGETS test_maps DESTINATION ${CMAKE_INSTALL_BINDIR})

	add_executable(torch_data_loader torch_data_loader.cpp)
	target_link_libraries(torch_data_loader PRIVATE ${dependencies_list})
	install(TARGETS torch_data_loader DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
