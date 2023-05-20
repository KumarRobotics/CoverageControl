#!/bin/bash

MAIN_DIR=$1
BUILD_DIR=${MAIN_DIR}/build

CMAKE_END_FLAGS="-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_FLAGS=-fPIC -DCMAKE_C_FLAGS=-fPIC"

InstallCGAL () {
	echo "Setting up CGAL"
	wget https://github.com/CGAL/cgal/releases/download/v5.5.2/CGAL-5.5.2-library.tar.xz -P ${MAIN_DIR}/src
	tar -xf ${MAIN_DIR}/src/CGAL-5.5.2-library.tar.xz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/CGAL-5.5.2 -B ${BUILD_DIR}/cgal ${CMAKE_END_FLAGS}
	cmake --install ${BUILD_DIR}/cgal
	if [ $? -eq 0 ]; then
		echo "cgal install succeeded"
	else
		echo "cgal install failed"
		exit 1
	fi
}

InstallGeoGraphicLib () {
	echo "Setting up geographiclib"
	wget https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.2.tar.gz -P ${MAIN_DIR}/src
	tar -xf ${MAIN_DIR}/src/v2.2.tar.gz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/geographiclib-2.2 -B ${BUILD_DIR}/geographiclib ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/geographiclib -j$(nproc)
	cmake --install ${BUILD_DIR}/geographiclib
	if [ $? -eq 0 ]; then
		echo "geographiclib install succeeded"
	else
		echo "geographiclib install failed"
		exit 1
	fi
}

InstallPybind11 () {
	echo "Setting up pybind11"
	wget https://github.com/pybind/pybind11/archive/refs/tags/v2.10.4.tar.gz -P ${MAIN_DIR}/src
	tar -xf ${MAIN_DIR}/src/v2.10.4.tar.gz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/pybind11-2.10.4 -B ${BUILD_DIR}/pybind11 ${CMAKE_END_FLAGS} -DPYBIND11_TEST=OFF
	cmake --build ${BUILD_DIR}/pybind11 -j$(nproc)
	cmake --install ${BUILD_DIR}/pybind11
	if [ $? -eq 0 ]; then
		echo "pybind11 install succeeded"
	else
		echo "pybind11 install failed"
		exit 1
	fi
}

InstallYamlCPP () {
	echo "Setting up yaml-cpp"
	git clone https://github.com/jbeder/yaml-cpp.git ${MAIN_DIR}/src/yaml-cpp
	env CFLAGS='-fPIC' CXXFLAGS='-fPIC' cmake -S ${MAIN_DIR}/src/yaml-cpp -B ${BUILD_DIR}/yaml-cpp -DYAML_BUILD_SHARED_LIBS=ON  ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/yaml-cpp -j$(nproc)
	if [ $? -ne 0 ]; then
		echo "YAML build failed"
	fi
	cmake --install ${BUILD_DIR}/yaml-cpp
	if [ $? -eq 0 ]; then
		echo "yaml-cpp install succeeded"
	else
		echo "yaml-cpp install failed"
		exit 1
	fi
}

InstallEigen3 () {
	echo "Setting up eigen3"
	wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz -P ${MAIN_DIR}/src
	tar -xf ${MAIN_DIR}/src/eigen-3.4.0.tar.gz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/eigen-3.4.0 -B ${BUILD_DIR}/eigen3 ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/eigen3
	cmake --install ${BUILD_DIR}/eigen3
	if [ $? -eq 0 ]; then
		echo "eigen3 install succeeded"
	else
		echo "eigen3 install failed"
		exit 1
	fi
}

InstallEigen3
InstallPybind11
InstallYamlCPP
InstallCGAL
InstallGeoGraphicLib
