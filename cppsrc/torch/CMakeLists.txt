cmake_minimum_required(VERSION 3.24)
project(CoverageControlTorch VERSION 0.3 LANGUAGES CXX)

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

find_package(Torch REQUIRED)
# find_package(TorchVision REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(CoverageControl REQUIRED)
get_filename_component(TORCH_LIBRARY_DIR "${TORCH_LIBRARIES}" DIRECTORY)
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
## CoverageControlTorch library ##
#################################

set(sources_list base.cpp)
list(TRANSFORM sources_list PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/src/")

set(dependencies_list CoverageControl::CoverageControl ${TORCH_LIBRARIES})

add_library(${PROJECT_NAME} SHARED ${sources_list})
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_BINARY_DIR} ${TORCH_INCLUDE_DIRS} ${CoverageControl_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE compiler_flags)
target_link_libraries(${PROJECT_NAME} PUBLIC ${dependencies_list})
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${ORIGIN};${CMAKE_INSTALL_LIBDIR};${TORCH_LIBRARY_DIR}")

if (WITH_CUDA)
	set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_ARCHITECTURES native POSITION_INDEPENDENT_CODE ON)
else()
	set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

target_include_directories(${PROJECT_NAME} INTERFACE
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(${PROJECT_NAME} INTERFACE ${dependencies_list})

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
