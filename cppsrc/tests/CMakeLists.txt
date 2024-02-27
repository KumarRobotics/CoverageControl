cmake_minimum_required(VERSION 3.16)
project(CoverageControlTests LANGUAGES CXX)

# Check if file VERSION exists
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../VERSION")
	file(READ "${CMAKE_CURRENT_SOURCE_DIR}/../../VERSION" VERSION)
	string(STRIP ${VERSION} VERSION)
	set(${PROJECT_NAME}_VERSION ${VERSION})
else()
	message(WARNING "VERSION file not found, using 0.0 as default version")
	set(${PROJECT_NAME}_VERSION 0.0)
endif()

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

option(WITH_CUDA "Build with CUDA support" ON)

if(WITH_CUDA)
	enable_language(CUDA)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

find_package(CoverageControl REQUIRED)

set(dependencies_list CoverageControl::CoverageControl)
if(NOT DEFINED WITH_TORCH)
	set(WITH_TORCH OFF)
endif()
if(WITH_TORCH)
	find_package(Torch REQUIRED)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
	get_filename_component(TORCH_LIBRARY_DIR "${TORCH_LIBRARIES}" DIRECTORY)
	set(CMAKE_INSTALL_RPATH "${ORIGIN};${CMAKE_INSTALL_LIBDIR};${TORCH_LIBRARY_DIR}")
endif()

###########
## Tests ##
###########


if(WITH_TORCH)
	find_package(CoverageControlTorch REQUIRED)
	set(dependencies_list ${dependencies_list} CoverageControl::CoverageControlTorch ${TORCH_LIBRARIES})
endif()

add_executable(simultaneous_explore_exploit simultaneous_explore_exploit.cpp)
target_link_libraries(simultaneous_explore_exploit PRIVATE CoverageControl::CoverageControl)
install(TARGETS simultaneous_explore_exploit DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(map_generation map_generation.cpp)
target_link_libraries(map_generation PRIVATE CoverageControl::CoverageControl)
install(TARGETS map_generation DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_executable(geo_transforms ${PROJECT_SOURCE_DIR}/test/geo_transforms.cpp)
# target_include_directories(geo_transforms PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/extern)
# target_link_libraries(geo_transforms PRIVATE Eigen3::Eigen CoverageControlTorch OpenMP::OpenMP_CXX ${GeographicLib_LIBRARIES})
# install(TARGETS geo_transforms DESTINATION ${CMAKE_INSTALL_BINDIR})

if(WITH_TORCH)
	add_executable(torch_test torch/torch_test.cpp)
	target_link_libraries(torch_test PRIVATE ${dependencies_list})
	install(TARGETS torch_test DESTINATION ${CMAKE_INSTALL_BINDIR})

	add_executable(torch_map_conversion torch/torch_map_conversion.cpp)
	target_link_libraries(torch_map_conversion PRIVATE ${dependencies_list})
	install(TARGETS torch_map_conversion DESTINATION ${CMAKE_INSTALL_BINDIR})

	add_executable(torch_scripting torch/torch_scripting.cpp)
	target_link_libraries(torch_scripting PRIVATE ${dependencies_list})
	install(TARGETS torch_scripting DESTINATION ${CMAKE_INSTALL_BINDIR})

	add_executable(torch_data_loader torch/torch_data_loader.cpp)
	target_link_libraries(torch_data_loader PRIVATE ${dependencies_list})
	install(TARGETS torch_data_loader DESTINATION ${CMAKE_INSTALL_BINDIR})

	add_executable(torch_normalization torch/torch_normalization.cpp)
	target_link_libraries(torch_normalization PRIVATE ${dependencies_list})
	install(TARGETS torch_normalization DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
