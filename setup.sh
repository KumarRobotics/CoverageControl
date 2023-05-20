#!/bin/bash
print_usage() {
	printf "bash $0 [-c (for clean)] [-i (for install)]\n"
}

while getopts 'uic' flag; do
	case "${flag}" in
		i) INSTALL=true;;
		c) CLEAN=true;;
		*) print_usage
			exit 1 ;;
esac
done

BUILD_DIR=${COVERAGECONTROL_WS}/build
INSTALL_DIR=${COVERAGECONTROL_WS}/install

mkdir -p ${COVERAGECONTROL_WS}/src

CMAKE_END_FLAGS="-DCMAKE_BUILD_TYPE=Release"

CleanBuild () {
	rm -rf ${BUILD_DIR}
	rm -rf ${INSTALL_DIR}
}

InstallCoverageControl () {
	cmake -S ${COVERAGECONTROL_WS}/src/CoverageControl/core -B ${BUILD_DIR}/CoverageControl ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/CoverageControl -j$(nproc)
	if [ $? -ne 0 ]; then
		echo "CoverageControl_core build failed"
		exit 1
	fi
	cmake --install ${BUILD_DIR}/CoverageControl
	if [ $? -ne 0 ]; then
		echo "CoverageControl_core install failed"
	fi

	cmake -S ${COVERAGECONTROL_WS}/src/CoverageControl/torch_cpp -B ${BUILD_DIR}/CoverageControlTorch ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/CoverageControlTorch -j$(nproc)
	if [ $? -ne 0 ]; then
		echo "CoverageControlTorch build failed"
		exit 1
	fi
	cmake --install ${BUILD_DIR}/CoverageControlTorch
	if [ $? -ne 0 ]; then
		echo "CoverageControlTorch install failed"
	fi
}

if [[ ${INSTALL} ]]
then
	InstallCoverageControl
fi

if [[ ${CLEAN} ]]
then
	echo "Cleaning build and install directories"
	CleanBuild
fi
