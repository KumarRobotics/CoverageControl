cmake_minimum_required(VERSION 3.15)

project(CoverageControlMain VERSION 0.3 LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)

find_package(Torch REQUIRED)
find_package(TorchVision REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS} -Wall -Wextra -Wshadow -Wformat=2 -Wunused -pedantic")

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(CGAL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CoverageControlCore REQUIRED)
find_package(CoverageControlTorch REQUIRED)

#################
## Executables ##
#################

add_library(compiler_flags INTERFACE)
target_compile_features(compiler_flags INTERFACE cxx_std_17)

set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
target_compile_options(compiler_flags INTERFACE
	"$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused;-pedantic>>"
	"$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
)
add_executable(data_generation data_generation.cpp)
target_link_libraries(data_generation PRIVATE compiler_flags CoverageControlCore CoverageControlTorch)
install(TARGETS data_generation DESTINATION ${CMAKE_INSTALL_BINDIR})
