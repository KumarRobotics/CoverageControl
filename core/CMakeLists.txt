cmake_minimum_required(VERSION 3.15)

project(CoverageControl VERSION 0.3 LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

configure_file(cmake/CoverageControlConfig.h.in CoverageControlConfig.h)

add_library(CoverageControl_compiler_flags INTERFACE)
target_compile_features(CoverageControl_compiler_flags INTERFACE cxx_std_17)

set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
target_compile_options(CoverageControl_compiler_flags INTERFACE
	"$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused;-pedantic>>"
	"$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
)

# list(APPEND CMAKE_PREFIX_PATH $ENV{COVERAGECONTROL_WS}/install)

# if(NOT DEFINED CMAKE_CUDA_STANDARD)
#     set(CMAKE_CUDA_STANDARD 17)
#     set(CMAKE_CUDA_STANDARD_REQUIRED ON)
# endif()

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CGAL REQUIRED)
find_package(GeographicLib REQUIRED)

#############################
## CoverageControl library ##
#############################

set(coveragecontrol_sources
	generate_world_map.cu
	polygon_utils.cpp
	parameters.cpp
	voronoi.cpp
	coverage_system.cpp
	plotter.cpp
	extern/Hungarian.cpp)
list(TRANSFORM coveragecontrol_sources PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/src/")

set(CoverageControl_dependencies Eigen3::Eigen m stdc++fs yaml-cpp OpenMP::OpenMP_CXX CGAL::CGAL pthread ${GeographicLib_LIBRARIES} boost_iostreams boost_system boost_filesystem)

add_library(CoverageControl_core SHARED ${coveragecontrol_sources})

add_library(CoverageControl INTERFACE)
target_include_directories(CoverageControl INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(CoverageControl INTERFACE ${CoverageControl_dependencies} CoverageControl_core)

target_include_directories(CoverageControl_core PRIVATE "${PROJECT_BINARY_DIR}")
target_link_libraries(CoverageControl_core PRIVATE ${CoverageControl_dependencies} CoverageControl_compiler_flags)
set_target_properties(CoverageControl_core PROPERTIES CUDA_ARCHITECTURES native POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS CoverageControl_core CoverageControl EXPORT CoverageControlTargets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

set(CMAKEPACKAGE_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/CoverageControl)

include(CMakePackageConfigHelpers)
# generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/CoverageControlConfig.cmake
	INSTALL_DESTINATION "${CMAKEPACKAGE_INSTALL_DIR}"
	NO_SET_AND_CHECK_MACRO
	NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/CoverageControlConfigVersion.cmake"
	VERSION "${Tutorial_VERSION_MAJOR}.${Tutorial_VERSION_MINOR}"
	COMPATIBILITY AnyNewerVersion
)

export (PACKAGE CoverageControl)
install(EXPORT CoverageControlTargets FILE CoverageControlTargets.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CoverageControl)

install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/CoverageControlConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/CoverageControlConfigVersion.cmake
	DESTINATION ${CMAKEPACKAGE_INSTALL_DIR}
)
