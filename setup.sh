#!/bin/bash
print_usage() {
	printf "bash $0 [-u (for update)] [-i (for install)]\n"
}

while getopts 'ui' flag; do
	case "${flag}" in
		u) UPDATE=true;;
		i) INSTALL=true;;
		*) print_usage
			exit 1 ;;
esac
done

WORKSPACE_DIR=${HOME}/CoverageControl_ws
BUILD_DIR=${WORKSPACE_DIR}/build
INSTALL_DIR=${WORKSPACE_DIR}/install

mkdir -p ${WORKSPACE_DIR}/src

InstallCGAL () {
	echo "Setting up CGAL"
	wget https://github.com/CGAL/cgal/releases/download/v5.5.1/CGAL-5.5.1-library.tar.xz -P ${WORKSPACE_DIR}/src
	tar -xf ${WORKSPACE_DIR}/src/CGAL-5.5.1-library.tar.xz -C ${WORKSPACE_DIR}/src/
	cmake -S ${WORKSPACE_DIR}/src/CGAL-5.5.1 -B ${BUILD_DIR}/cgal -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
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
	wget https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.1.2.tar.gz -P ${WORKSPACE_DIR}/src
	tar -xf ${WORKSPACE_DIR}/src/v2.1.2.tar.gz -C ${WORKSPACE_DIR}/src/
	cmake -S ${WORKSPACE_DIR}/src/geographiclib-2.1.2 -B ${BUILD_DIR}/geographiclib -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
	cmake --build ${BUILD_DIR}/geographiclib
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
	wget https://github.com/pybind/pybind11/archive/refs/tags/v2.10.1.tar.gz -P ${WORKSPACE_DIR}/src
	tar -xf ${WORKSPACE_DIR}/src/v2.10.1.tar.gz -C ${WORKSPACE_DIR}/src/
	cmake -S ${WORKSPACE_DIR}/src/pybind11-2.10.1 -B ${BUILD_DIR}/pybind11 -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
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
	git clone git@github.com:jbeder/yaml-cpp.git ${WORKSPACE_DIR}/src/yaml-cpp
	cmake -S ${WORKSPACE_DIR}/src/yaml-cpp -B ${BUILD_DIR}/yaml-cpp -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DYAML_BUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
	cmake --build ${BUILD_DIR}/yaml-cpp -j4
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
	wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz -P ${WORKSPACE_DIR}/src
	tar -xf ${WORKSPACE_DIR}/src/eigen-3.4.0.tar.gz -C ${WORKSPACE_DIR}/src/
	cmake -S ${WORKSPACE_DIR}/src/eigen-3.4.0 -B ${BUILD_DIR}/eigen3 -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
	cmake --install ${BUILD_DIR}/eigen3
	if [ $? -eq 0 ]; then
		echo "eigen3 install succeeded"
	else
		echo "eigen3 install failed"
		exit 1
	fi
}

InstallCoverageControl () {
	echo "Setting up CoverageControl"
	git clone git@github.com:AgarwalSaurav/CoverageControl.git ${WORKSPACE_DIR}/src/CoverageControl
	cmake -S ${WORKSPACE_DIR}/src/CoverageControl -B ${BUILD_DIR}/CoverageControl -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
	cmake --build ${BUILD_DIR}/CoverageControl
	if [ $? -ne 0 ]; then
		echo "CoverageControl build failed"
	fi
	cmake --install ${BUILD_DIR}/CoverageControl
	if [ $? -eq 0 ]; then
		echo "CoverageControl install succeeded"
	else
		echo "CoverageControl install failed"
		exit 1
	fi
}

UpdateCoverageControl () {
	# Run the following commands to update after a change in the repository
	# The CoverageControl repository is located in ${WORKSPACE_DIR}/src/CoverageControl
	cmake -S ${WORKSPACE_DIR}/src/CoverageControl -B ${BUILD_DIR}/CoverageControl -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
	cmake --build ${BUILD_DIR}/CoverageControl
	if [ $? -ne 0 ]; then
		echo "CoverageControl build failed"
		exit 1
	fi
	cmake --install ${BUILD_DIR}/CoverageControl
	if [ $? -ne 0 ]; then
		echo "CoverageControl install failed"
	fi
}

if [[ ${INSTALL} ]]
then
	echo "Installing all dependencies"
	InstallEigen3
	InstallPybind11
	InstallYamlCPP
	InstallCGAL
	InstallGeoGraphicLib
	InstallCoverageControl
fi

if [[ ${UPDATE} ]]
then
	echo "Updating Coverage Control"
	UpdateCoverageControl
fi
