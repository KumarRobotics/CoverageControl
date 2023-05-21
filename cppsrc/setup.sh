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

CMAKE_END_FLAGS="-DCMAKE_BUILD_TYPE=RelWithDebInfo"

CleanBuild () {
	rm -rf ${BUILD_DIR}
	rm -rf ${INSTALL_DIR}
}

InstallCoverageControl () {
	cmake -S ${COVERAGECONTROL_WS}/src/CoverageControl/cppsrc/core -B ${BUILD_DIR}/CoverageControlCore ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/CoverageControlCore -j$(nproc)
	if [ $? -ne 0 ]; then
		echo "CoverageControlCore build failed"
		exit 1
	fi
	cmake --install ${BUILD_DIR}/CoverageControlCore
	if [ $? -ne 0 ]; then
		echo "CoverageControlCore install failed"
	fi

	echo "Successfully built and installed CoverageControlCore"

	cmake -S ${COVERAGECONTROL_WS}/src/CoverageControl/cppsrc/torch -B ${BUILD_DIR}/CoverageControlTorch ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/CoverageControlTorch -j$(nproc)
	if [ $? -ne 0 ]; then
		echo "CoverageControlTorch build failed"
		exit 1
	fi
	cmake --install ${BUILD_DIR}/CoverageControlTorch
	if [ $? -ne 0 ]; then
		echo "CoverageControlTorch install failed"
	fi

	echo "Successfully built and installed CoverageControlTorch"

	cmake -S ${COVERAGECONTROL_WS}/src/CoverageControl/cppsrc/tests -B ${BUILD_DIR}/CoverageControlTests ${CMAKE_END_FLAGS} -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
	cmake --build ${BUILD_DIR}/CoverageControlTests -j$(nproc)
	if [ $? -ne 0 ]; then
		echo "CoverageControlTests build failed"
		exit 1
	fi
	cmake --install ${BUILD_DIR}/CoverageControlTests
	if [ $? -ne 0 ]; then
		echo "CoverageControlTests install failed"
	fi

	echo "Successfully built and installed CoverageControlTests"
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
