cmake_minimum_required(VERSION 3.28...4.4)

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
  include(CheckLanguage)
  check_language(CUDA)
  if(NOT CMAKE_CUDA_COMPILER)
    find_package(CUDAToolkit)
    if(CUDAToolkit_FOUND)
      enable_language(CUDA)
    else()
      message(WARNING "No CUDA compiler found, disabling CUDA support")
      set(COVERAGECONTROL_WITH_CUDA OFF)
    endif()
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


###########################
#### OpenMP dependency ####
###########################
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(OpenMP_CXX_FLAGS "-fopenmp=libomp")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY omp)
  endif()
endif()

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost CONFIG REQUIRED COMPONENTS iostreams)

include(FetchContent)
set(FETCHCONTENT_QUIET FALSE) # show download progress

###########################
#### Eigen3 dependency ####
###########################
# Always fetched, never taken from the system: Eigen types cross this
# library's ABI boundary, so the headers bundled into the install below must
# be the exact version the library was compiled with.
FetchContent_Declare(Eigen3
  URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz
  URL_HASH SHA256=e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec
  EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(Eigen3)

###########################
#### CGAL dependency ####
###########################
FetchContent_Declare(CGAL
  URL https://github.com/CGAL/cgal/releases/download/v6.2/CGAL-6.2.tar.xz
  URL_HASH SHA256=fbc32816745e871a5cbdeb6245317e9dbf10ae1a957b0ab1edb00b4fde00ba8d
  # CGAL's release build system only works as the top-level project, so the
  # non-existent SOURCE_SUBDIR makes this a download-only dependency.
  SOURCE_SUBDIR use-find-package-instead
  FIND_PACKAGE_ARGS 6.2 EXACT
)
FetchContent_MakeAvailable(CGAL)
if(NOT CGAL_FOUND)
  include("${cgal_SOURCE_DIR}/CGALConfig.cmake")
endif()

###########################

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

# BUILD_INTERFACE keeps Eigen out of the install export set (it may be a
# non-exportable subproject target when fetched); the installed package
# config re-finds Eigen3 and links it to the imported target itself.
set(dependencies_list "$<BUILD_INTERFACE:Eigen3::Eigen>" OpenMP::OpenMP_CXX)

add_library(${PROJECT_NAME} SHARED ${sources_list})
target_include_directories(${PROJECT_NAME} PRIVATE "${PROJECT_BINARY_DIR}")
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_INSTALL_INCLUDEDIR}")
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(${PROJECT_NAME} PRIVATE compiler_flags CGAL::CGAL Boost::headers Boost::iostreams)
target_link_libraries(${PROJECT_NAME} PUBLIC ${dependencies_list})
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")

# Never re-export symbols pulled in from static archives. A toolchain that
# resolves -lstdc++ to libstdc++.a (Arch gcc 16.1.1-r346-3 shipped no
# libstdc++.so in gcc's libdir) would otherwise leave unversioned std::
# symbols here that interpose on the process's real libstdc++.so.6, so
# objects built by one copy get destroyed by the other -- torch's
# std::stringstream aborts with "free(): invalid size" on import.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_link_options(${PROJECT_NAME} PRIVATE "LINKER:--exclude-libs,ALL")
endif()

if(COVERAGECONTROL_WITH_CUDA)
  # "all" triggers a CCCL preprocessor overflow in CUDA >= 12.9 thrust
  # headers (NVIDIA/cccl#4967); all-major still covers every GPU generation.
  set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_ARCHITECTURES all-major POSITION_INDEPENDENT_CODE ON)
else()
  set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INCLUDE_OUTPUT_DIRECTORY}>")

# The cgal/ headers are implementation detail: nothing reachable from the
# public API includes them, and installing them would require shipping CGAL.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INCLUDE_OUTPUT_DIRECTORY}
  PATTERN "cgal" EXCLUDE)
install(DIRECTORY ${eigen3_SOURCE_DIR}/Eigen DESTINATION ${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/eigen3)
install(FILES
  ${eigen3_SOURCE_DIR}/COPYING.APACHE
  ${eigen3_SOURCE_DIR}/COPYING.BSD
  ${eigen3_SOURCE_DIR}/COPYING.MINPACK
  ${eigen3_SOURCE_DIR}/COPYING.MPL2
  ${eigen3_SOURCE_DIR}/COPYING.README
  DESTINATION ${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/eigen3)
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
