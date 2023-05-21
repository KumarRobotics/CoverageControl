cmake_minimum_required(VERSION 3.15)
project(CoverageControlCore VERSION 0.3 LANGUAGES CUDA CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CGAL REQUIRED)
find_package(GeographicLib REQUIRED)

configure_file(cmake/CoverageControlCoreConfig.h.in CoverageControlCoreConfig.h)

add_library(compiler_flags INTERFACE)
target_compile_features(compiler_flags INTERFACE cxx_std_17)

set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
target_compile_options(compiler_flags INTERFACE
	"$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused;-pedantic>>"
	"$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
)


#################################
## CoverageControlCore library ##
#################################

set(sources_list
	generate_world_map.cu
	polygon_utils.cpp
	parameters.cpp
	voronoi.cpp
	coverage_system.cpp
	plotter.cpp
	extern/Hungarian.cpp)
list(TRANSFORM sources_list PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/src/")

set(dependencies_list Eigen3::Eigen m stdc++fs yaml-cpp OpenMP::OpenMP_CXX CGAL::CGAL pthread ${GeographicLib_LIBRARIES} boost_iostreams boost_system boost_filesystem)

add_library(CoverageControl_core SHARED ${sources_list})

add_library(CoverageControlCore INTERFACE)
target_include_directories(CoverageControlCore INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(CoverageControlCore INTERFACE ${dependencies_list} CoverageControl_core)

target_include_directories(CoverageControl_core PRIVATE "${PROJECT_BINARY_DIR}")
target_link_libraries(CoverageControl_core PRIVATE compiler_flags)
target_link_libraries(CoverageControl_core PUBLIC ${dependencies_list})
set_target_properties(CoverageControl_core PROPERTIES CUDA_ARCHITECTURES native POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS CoverageControl_core CoverageControlCore EXPORT CoverageControlCoreTargets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

set(CMAKEPACKAGE_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/CoverageControlCore)

include(CMakePackageConfigHelpers)
# generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/CoverageControlCoreConfig.cmake
	INSTALL_DESTINATION "${CMAKEPACKAGE_INSTALL_DIR}"
	NO_SET_AND_CHECK_MACRO
	NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/CoverageControlCoreConfigVersion.cmake"
	VERSION "${CoverageControlCore_VERSION_MAJOR}.${CoverageControlCore_VERSION_MINOR}"
	COMPATIBILITY AnyNewerVersion
)

export (PACKAGE CoverageControlCore)
install(EXPORT CoverageControlCoreTargets FILE CoverageControlCoreTargets.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CoverageControlCore)

install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/CoverageControlCoreConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/CoverageControlCoreConfigVersion.cmake
	DESTINATION ${CMAKEPACKAGE_INSTALL_DIR}
)
