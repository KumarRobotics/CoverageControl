cmake_minimum_required(VERSION 3.16)

if(DEFINED SKBUILD_PROJECT_VERSION)
  set(PROJECT_VERSION ${SKBUILD_PROJECT_VERSION})
else()
  include(FetchContent)
  FetchContent_Declare(CMakeExtraUtils
    GIT_REPOSITORY https://github.com/LecrisUT/CMakeExtraUtils
    GIT_TAG v0.4.1)
  FetchContent_MakeAvailable(CMakeExtraUtils)

  include(DynamicVersion)
  dynamic_version(
    PROJECT_PREFIX "CoverageControl_"
    GIT_ARCHIVAL_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.git_archival.txt"
    FALLBACK_VERSION 0.0.1
  )
endif()

project(CoverageControl VERSION ${PROJECT_VERSION} LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(COVERAGECONTROL_WITH_CUDA "Enable CUDA support" ON)

if(COVERAGECONTROL_WITH_CUDA)
  cmake_minimum_required(VERSION 3.24)
  include(CheckLanguage)
  check_language(CUDA)
  if(NOT CMAKE_CUDA_COMPILER)
    message(WARNING "No CUDA compiler found, disabling CUDA support")
    set(COVERAGECONTROL_WITH_CUDA OFF)
  else()
    enable_language(CUDA)
  endif()
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_COLOR_DIAGNOSTICS ON)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  if(UNIX)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_LIBDIR})
  else()
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_BINDIR})
  endif()
endif()
if(NOT CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
  if(UNIX)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_INSTALL_LIBDIR})
  else()
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_INSTALL_BINDIR})
  endif()
endif()
if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_BINDIR})
endif()
if(NOT CMAKE_INCLUDE_OUTPUT_DIRECTORY)
  set(CMAKE_INCLUDE_OUTPUT_DIRECTORY ${CMAKE_INSTALL_INCLUDEDIR})
endif()

set(CMAKEPACKAGE_INSTALL_DIR
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/${PROJECT_NAME}"
)


find_package(OpenMP REQUIRED)

###########################
#### Eigen3 dependency ####
###########################
find_package(Eigen3 3.4 QUIET)
if(NOT Eigen3_FOUND OR Eigen3_VERSION VERSION_LESS 3.4)
  message(STATUS "Eigen3 3.4 not found, fetching it")
  include(FetchContent)
  FetchContent_Declare(Eigen3
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  )
  set(FETCHCONTENT_QUIET FALSE) # show progress dialog
  FetchContent_Populate(Eigen3)   # finish fetching and unpacking before continuing
  add_subdirectory(${FETCHCONTENT_BASE_DIR}/eigen3-src ${CMAKE_CURRENT_BINARY_DIR}/eigen3)
endif()

###########################
#### CGAL dependency ####
###########################
find_package(CGAL 5.6.1 QUIET)
if(NOT CGAL_FOUND OR CGAL_VERSION VERSION_LESS 5.6.1)
  message(STATUS "CGAL 5.6.1 not found, fetching it")
  include(FetchContent)
  FetchContent_Declare(CGAL
    URL https://github.com/CGAL/cgal/releases/download/v5.6.1/CGAL-5.6.1.tar.xz
    URL_HASH SHA256=cdb15e7ee31e0663589d3107a79988a37b7b1719df3d24f2058545d1bcdd5837
  )
  set(FETCHCONTENT_QUIET FALSE) # show progress dialog
  FetchContent_Populate(CGAL)   # finish fetching and unpacking before continuing
  set(CGAL_DIR "${FETCHCONTENT_BASE_DIR}/cgal-src")
  find_package(CGAL 5.6 REQUIRED)
endif()
###########################

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS iostreams)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.h.in
  ${PROJECT_NAME}/Config.h
)

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
  world_idf.cpp
  coverage_system.cpp
  plotter.cpp
  cuda_utils.cpp
  extern/Hungarian.cpp)

if(COVERAGECONTROL_WITH_CUDA)
  list(APPEND sources_list
    cuda/generate_world_map.cu
    cuda/cuda_utils.cu)
endif()
list(TRANSFORM sources_list PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/src/")

set(dependencies_list Eigen3::Eigen OpenMP::OpenMP_CXX)

add_library(${PROJECT_NAME} SHARED ${sources_list})
target_include_directories(${PROJECT_NAME} PRIVATE "${PROJECT_BINARY_DIR}")
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_INSTALL_INCLUDEDIR}")
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(${PROJECT_NAME} PRIVATE compiler_flags CGAL::CGAL Boost::boost Boost::iostreams)
target_link_libraries(${PROJECT_NAME} PUBLIC ${dependencies_list})
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")

if(COVERAGECONTROL_WITH_CUDA)
  set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_ARCHITECTURES all POSITION_INDEPENDENT_CODE ON)
else()
  set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INCLUDE_OUTPUT_DIRECTORY}>")

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INCLUDE_OUTPUT_DIRECTORY})
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets LIBRARY DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ARCHIVE DESTINATION ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}/Config.h" DESTINATION ${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/${PROJECT_NAME})

include(CMakePackageConfigHelpers)
# generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "${CMAKEPACKAGE_INSTALL_DIR}")

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}"
  COMPATIBILITY AnyNewerVersion)

export(PACKAGE ${PROJECT_NAME})

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKEPACKAGE_INSTALL_DIR})

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE CoverageControl::
  DESTINATION ${CMAKEPACKAGE_INSTALL_DIR})
