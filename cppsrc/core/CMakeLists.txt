cmake_minimum_required(VERSION 3.16)
project(CoverageControl LANGUAGES CXX)

# Check if file VERSION exists
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../VERSION")
	file(READ "${CMAKE_CURRENT_SOURCE_DIR}/../../VERSION" VERSION)
	string(STRIP ${VERSION} VERSION)
	set(${PROJECT_NAME}_VERSION ${VERSION})
else()
	message(WARNING "VERSION file not found, using 0.0 as default version")
	set(${PROJECT_NAME}_VERSION 0.0)
endif()

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

option(WITH_CUDA "Enable CUDA support" ON)

if (WITH_CUDA)
	cmake_minimum_required(VERSION 3.24)
	enable_language(CUDA)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

find_package(OpenMP REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(CGAL 5.6 REQUIRED)

configure_file(cmake/${PROJECT_NAME}Config.h.in ${PROJECT_NAME}Config.h)

add_library(compiler_flags INTERFACE)
target_compile_features(compiler_flags INTERFACE cxx_std_17)

set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
target_compile_options(compiler_flags INTERFACE
	"$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused;-pedantic>>"
	"$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
)

#################################
## CoverageControl library ##
#################################

set(sources_list
	polygon_utils.cpp
	parameters.cpp
	voronoi.cpp
	coverage_system.cpp
	plotter.cpp
	extern/Hungarian.cpp)

if (WITH_CUDA)
	list(APPEND sources_list
	generate_world_map.cu)
endif()
list(TRANSFORM sources_list PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/src/")

set(dependencies_list Eigen3::Eigen m stdc++fs OpenMP::OpenMP_CXX pthread boost_iostreams boost_system boost_filesystem)

add_library(${PROJECT_NAME} SHARED ${sources_list})
target_include_directories(${PROJECT_NAME} PRIVATE "${PROJECT_BINARY_DIR}")
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_INSTALL_INCLUDEDIR}")
target_link_libraries(${PROJECT_NAME} PRIVATE compiler_flags CGAL::CGAL)
target_link_libraries(${PROJECT_NAME} PUBLIC ${dependencies_list})
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}")

if (WITH_CUDA)
	set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_ARCHITECTURES native POSITION_INDEPENDENT_CODE ON CUDA_SEPARABLE_COMPILATION ON)
	else()
		set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

target_include_directories(${PROJECT_NAME} INTERFACE
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.h" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(CMAKEPACKAGE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/cmake/${PROJECT_NAME}")

include(CMakePackageConfigHelpers)
# generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
	INSTALL_DESTINATION "${CMAKEPACKAGE_INSTALL_DIR}")

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
	VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}"
	COMPATIBILITY AnyNewerVersion)

export (PACKAGE ${PROJECT_NAME})

install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
	DESTINATION ${CMAKEPACKAGE_INSTALL_DIR})

install(EXPORT ${PROJECT_NAME}Targets
	FILE ${PROJECT_NAME}Targets.cmake
	NAMESPACE CoverageControl::
	DESTINATION ${CMAKEPACKAGE_INSTALL_DIR})
