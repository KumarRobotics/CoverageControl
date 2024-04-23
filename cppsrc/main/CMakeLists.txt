cmake_minimum_required(VERSION 3.16)
project(CoverageControlMain LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

if(NOT DEFINED WITH_TORCH)
	set(WITH_TORCH OFF)
endif()
if(WITH_TORCH)
	find_package(Torch REQUIRED)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
	get_filename_component(TORCH_LIBRARY_DIR "${TORCH_LIBRARIES}" DIRECTORY)
	set(CMAKE_INSTALL_RPATH "${ORIGIN};${CMAKE_INSTALL_LIBDIR};${TORCH_LIBRARY_DIR}")
endif()

find_package(CoverageControl REQUIRED)

if(WITH_TORCH)
	find_package(CoverageControlTorch REQUIRED)
endif()

add_library(compiler_flags INTERFACE)
target_compile_features(compiler_flags INTERFACE cxx_std_17)

set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
target_compile_options(compiler_flags INTERFACE
	"$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused;-pedantic>>"
	"$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
)

#################
## Executables ##
#################

add_executable(coverage_algorithm coverage_algorithm.cpp)
target_link_libraries(coverage_algorithm PRIVATE CoverageControl::CoverageControl)
install(TARGETS coverage_algorithm DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(world_idf world_idf.cpp)
target_link_libraries(world_idf PRIVATE CoverageControl::CoverageControl)
install(TARGETS world_idf DESTINATION ${CMAKE_INSTALL_BINDIR})

# if(WITH_TORCH)
# 	add_executable(eval_dist_gnn torch/eval_dist_gnn.cpp)
# 	target_link_libraries(eval_dist_gnn PRIVATE compiler_flags CoverageControl CoverageControlTorch ${CUDA_cusparse_LIBRARY})
# 	install(TARGETS eval_dist_gnn DESTINATION ${CMAKE_INSTALL_BINDIR})
# endif()
