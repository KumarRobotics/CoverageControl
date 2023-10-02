cmake_minimum_required(VERSION 3.15)
project(CoverageControlTorch VERSION 0.3 LANGUAGES CUDA CXX)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)

find_package(Torch REQUIRED)
# find_package(TorchVision REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(OpenCV REQUIRED)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CGAL REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(CoverageControlCore REQUIRED)

configure_file(cmake/CoverageControlTorchConfig.h.in CoverageControlTorchConfig.h)

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

set(dependencies_list CoverageControlCore ${TORCH_LIBRARIES})

add_library(CoverageControl_torch SHARED ${sources_list})

add_library(CoverageControlTorch INTERFACE)
target_include_directories(CoverageControlTorch INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(CoverageControlTorch INTERFACE ${dependencies_list} CoverageControlCore CoverageControl_torch)

target_include_directories(CoverageControl_torch PRIVATE ${PROJECT_BINARY_DIR} ${TORCH_INCLUDE_DIRS})
target_link_libraries(CoverageControl_torch PRIVATE compiler_flags)
target_link_libraries(CoverageControl_torch PUBLIC ${dependencies_list})
set_target_properties(CoverageControl_torch PROPERTIES CUDA_ARCHITECTURES native POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS CoverageControl_torch CoverageControlTorch EXPORT CoverageControlTorchTargets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

set(CMAKEPACKAGE_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/CoverageControlTorch)

include(CMakePackageConfigHelpers)
# generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/CoverageControlTorchConfig.cmake
	INSTALL_DESTINATION "${CMAKEPACKAGE_INSTALL_DIR}"
	NO_SET_AND_CHECK_MACRO
	NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/CoverageControlTorchConfigVersion.cmake"
	VERSION "${CoverageControlTorch_VERSION_MAJOR}.${CoverageControlTorch_VERSION_MINOR}"
	COMPATIBILITY AnyNewerVersion
)

export (PACKAGE CoverageControlTorch)
install(EXPORT CoverageControlTorchTargets FILE CoverageControlTorchTargets.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CoverageControlTorch)

install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/CoverageControlTorchConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/CoverageControlTorchConfigVersion.cmake
	DESTINATION ${CMAKEPACKAGE_INSTALL_DIR}
)
