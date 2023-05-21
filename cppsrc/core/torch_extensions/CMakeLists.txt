cmake_minimum_required(VERSION 3.12)

project(CoverageControl VERSION 0.3 LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)


find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(CGAL REQUIRED)
find_package(GeographicLib REQUIRED)

find_package(Torch REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS} -Wall -pedantic")
# add_executable(example-app example-app.cpp)
# target_link_libraries(example-app ${TORCH_LIBRARIES} CoverageControl)

pybind11_add_module(pyCoverageControl MODULE extensions.cpp)
target_include_directories(pyCoverageControl PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/extern ${pybind11_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} /root/CoverageControl_ws/install/include ${TORCH_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS}})
target_link_libraries(pyCoverageControl PRIVATE CoverageControl ${PYTHON_LIBRARIES} Eigen3::Eigen m yaml-cpp OpenMP::OpenMP_CXX CGAL::CGAL pthread ${GeographicLib_LIBRARIES} boost_iostreams boost_system boost_filesystem ${TORCH_LIBRARIES})
target_link_directories(pyCoverageControl PRIVATE ${CMAKE_INSTALL_LIBDIR})
