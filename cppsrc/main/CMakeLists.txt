cmake_minimum_required(VERSION 3.24)

project(CoverageControlMain VERSION 0.3 LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)


if(NOT DEFINED WITH_TORCH)
	set(WITH_TORCH OFF)
endif()
if(WITH_TORCH)
	find_package(Torch REQUIRED)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
endif()

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(CoverageControlCore REQUIRED)

if(WITH_TORCH)
	find_package(CoverageControlTorch REQUIRED)
endif()


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

add_executable(coverage_algorithm coverage_algorithm.cpp)
target_link_libraries(coverage_algorithm PRIVATE CoverageControlCore)
install(TARGETS coverage_algorithm DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_executable(data_generation data_generation.cpp)
# target_link_libraries(data_generation PRIVATE compiler_flags CoverageControlCore CoverageControlTorch)
# install(TARGETS data_generation DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_executable(train_cnn train_cnn.cpp)
# target_link_libraries(train_cnn PRIVATE compiler_flags CoverageControlCore CoverageControlTorch)
# install(TARGETS train_cnn DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_executable(test_cnn test_cnn.cpp)
# target_link_libraries(test_cnn PRIVATE compiler_flags CoverageControlCore CoverageControlTorch)
# install(TARGETS test_cnn DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_executable(train_gnn train_gnn.cpp)
# target_link_libraries(train_gnn PRIVATE compiler_flags CoverageControlCore CoverageControlTorch ${CUDA_cusparse_LIBRARY})
# install(TARGETS train_gnn DESTINATION ${CMAKE_INSTALL_BINDIR})

# add_executable(eval_gnn eval_gnn.cpp)
# target_link_libraries(eval_gnn PRIVATE compiler_flags CoverageControlCore CoverageControlTorch ${CUDA_cusparse_LIBRARY})
# install(TARGETS eval_gnn DESTINATION ${CMAKE_INSTALL_BINDIR})

if(WITH_TORCH)
	add_executable(eval_dist_gnn eval_dist_gnn.cpp)
	target_link_libraries(eval_dist_gnn PRIVATE compiler_flags CoverageControlCore CoverageControlTorch ${CUDA_cusparse_LIBRARY})
	install(TARGETS eval_dist_gnn DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
