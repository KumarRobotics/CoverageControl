cmake_minimum_required(VERSION 3.16)
if(NOT DEFINED SKBUILD_PROJECT_NAME)
	set(PROJECT_NAME "CoverageControl")
else()
	set(PROJECT_NAME ${SKBUILD_PROJECT_NAME})
endif()
project(${PROJECT_NAME} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS True)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)

find_package(Python COMPONENTS Interpreter Development)
if(NOT ${SKBUILD_STATE} STREQUAL "sdist"
		AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pybind11/CMakeLists.txt")
	message(STATUS "Using integrated pybind11")
	add_subdirectory(third_party/pybind11)
else()
	message(STATUS "Using pybind11 via FetchContent")
	FetchContent_Declare(
		pybind11
		GIT_REPOSITORY https://github.com/pybind/pybind11.git
		GIT_TAG v2.12.0
	)
	FetchContent_MakeAvailable(pybind11)
endif()

set(PYBIND11_NEWPYTHON True)
# find_package(pybind11 CONFIG REQUIRED)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/lib")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_NAME}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_NAME}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_NAME}/lib)
set(CMAKE_INCLUDE_OUTPUT_DIRECTORY ${PROJECT_NAME}/include)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/cppsrc/core)

pybind11_add_module(_core MODULE cppsrc/python_bindings/python_binds.cpp)
target_include_directories(_core PUBLIC
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/cppsrc/core/include>"
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/cppsrc/core>")

target_link_libraries(_core PUBLIC CoverageControl)
install(TARGETS _core LIBRARY DESTINATION ${PROJECT_NAME})
