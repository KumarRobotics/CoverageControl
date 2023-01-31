#!/bin/bash
print_usage() {
	printf "bash $0 [-u (for update)] [-i (for install)]\n"
}

while getopts 'uic' flag; do
	case "${flag}" in
		u) UPDATE=true;;
		i) INSTALL=true;;
		c) CLEAN=true;;
		*) print_usage
			exit 1 ;;
esac
done

BUILD_DIR=${COVERAGECONTROL_WS}/build
INSTALL_DIR=${COVERAGECONTROL_WS}/install

mkdir -p ${COVERAGECONTROL_WS}/src

InstallCGAL () {
	echo "Setting up CGAL"
	wget https://github.com/CGAL/cgal/releases/download/v5.5.1/CGAL-5.5.1-library.tar.xz -P ${COVERAGECONTROL_WS}/src
	tar -xf ${COVERAGECONTROL_WS}/src/CGAL-5.5.1-library.tar.xz -C ${COVERAGECONTROL_WS}/src/
	cmake -S ${COVERAGECONTROL_WS}/src/CGAL-5.5.1 -B ${BUILD_DIR}/cgal -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
	cmake --install ${BUILD_DIR}/cgal
	if [ $? -eq 0 ]; then
		echo "cgal install succeeded"
	else
		echo "cgal install failed"
		exit 1
	fi
	rm ${COVERAGECONTROL_WS}/src/CGAL-5.5.1-library.tar.xz
	rm -rf ${COVERAGECONTROL_WS}/src/CGAL-5.5.1
}

InstallGeoGraphicLib () {
	echo "Setting up geographiclib"
	wget https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.1.2.tar.gz -P ${COVERAGECONTROL_WS}/src
	tar -xf ${COVERAGECONTROL_WS}/src/v2.1.2.tar.gz -C ${COVERAGECONTROL_WS}/src/
	cmake -S ${COVERAGECONTROL_WS}/src/geographiclib-2.1.2 -B ${BUILD_DIR}/geographiclib -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
	cmake --build ${BUILD_DIR}/geographiclib
	cmake --install ${BUILD_DIR}/geographiclib
	if [ $? -eq 0 ]; then
		echo "geographiclib install succeeded"
	else
		echo "geographiclib install failed"
		exit 1
	fi
	rm ${COVERAGECONTROL_WS}/src/v2.1.2.tar.gz
	rm -rf ${COVERAGECONTROL_WS}/src/geographiclib-2.1.2
}

InstallPybind11 () {
	echo "Setting up pybind11"
	wget https://github.com/pybind/pybind11/archive/refs/tags/v2.10.1.tar.gz -P ${COVERAGECONTROL_WS}/src
	tar -xf ${COVERAGECONTROL_WS}/src/v2.10.1.tar.gz -C ${COVERAGECONTROL_WS}/src/
	cmake -S ${COVERAGECONTROL_WS}/src/pybind11-2.10.1 -B ${BUILD_DIR}/pybind11 -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
	cmake --install ${BUILD_DIR}/pybind11
	if [ $? -eq 0 ]; then
		echo "pybind11 install succeeded"
	else
		echo "pybind11 install failed"
		exit 1
	fi
	rm ${COVERAGECONTROL_WS}/src/v2.10.1.tar.gz
	rm -rf ${COVERAGECONTROL_WS}/src/pybind11-2.10.1
}

InstallYamlCPP () {
	echo "Setting up yaml-cpp"
	git clone https://github.com/jbeder/yaml-cpp.git ${COVERAGECONTROL_WS}/src/yaml-cpp
	cmake -S ${COVERAGECONTROL_WS}/src/yaml-cpp -B ${BUILD_DIR}/yaml-cpp -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DYAML_BUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
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
	rm -rf ${COVERAGECONTROL_WS}/src/yaml-cpp
}

InstallEigen3 () {
	echo "Setting up eigen3"
	wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz -P ${COVERAGECONTROL_WS}/src
	tar -xf ${COVERAGECONTROL_WS}/src/eigen-3.4.0.tar.gz -C ${COVERAGECONTROL_WS}/src/
	cmake -S ${COVERAGECONTROL_WS}/src/eigen-3.4.0 -B ${BUILD_DIR}/eigen3 -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
	cmake --install ${BUILD_DIR}/eigen3
	if [ $? -eq 0 ]; then
		echo "eigen3 install succeeded"
	else
		echo "eigen3 install failed"
		exit 1
	fi
	rm ${COVERAGECONTROL_WS}/src/eigen-3.4.0.tar.gz
	rm -rf ${COVERAGECONTROL_WS}/src/eigen-3.4.0
}

CleanBuild () {
	rm -rf ${BUILD_DIR}
	rm -rf ${INSTALL_DIR}
	rm -rf ${COVERAGECONTROL_WS}/src/CoverageControl/build
	rm -rf ${COVERAGECONTROL_WS}/src/CoverageControl/pyCoverageControl.egg-info
}

UpdateCoverageControl () {
	# Run the following commands to update after a change in the repository
	# The CoverageControl repository is located in ${COVERAGECONTROL_WS}/src/CoverageControl
	cmake -S ${COVERAGECONTROL_WS}/src/CoverageControl -B ${BUILD_DIR}/CoverageControl -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
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
	UpdateCoverageControl
fi

if [[ ${UPDATE} ]]
then
	echo "Updating Coverage Control"
	UpdateCoverageControl
fi

if [[ ${CLEAN} ]]
then
	echo "Cleaning build and install directories"
	CleanBuild
fi
