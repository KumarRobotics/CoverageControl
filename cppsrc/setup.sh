#!/usr/bin/env bash
print_usage() {
	printf "bash $0 [-i <for install>] [-t <for torch>] [-p <for python>] [-d <workspace_dir>]\n"
}

# Get directory of script
# https://stackoverflow.com/questions/59895/getting-the-source-directory-of-a-bash-script-from-within
SOURCE=${BASH_SOURCE[0]}
while [ -L "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )
  SOURCE=$(readlink "$SOURCE")
  [[ $SOURCE != /* ]] && SOURCE=$DIR/$SOURCE # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )

WITH_TORCH=0
while getopts 'd:ictpg' flag; do
	case "${flag}" in
		i) INSTALL=true;;
		t) WITH_TORCH=ON;;
		p) WITH_PYTHON=true;;
		d) WS_DIR=${OPTARG};;
		g) GLOBAL=true;;
		*) print_usage
			exit 1 ;;
	esac
done

# if -d was given then set build_dir and install_dir to that

CMAKE_END_FLAGS="-DCMAKE_BUILD_TYPE=Release"
if [[ ${WITH_TORCH} == "ON" ]]
then
	CMAKE_END_FLAGS="${CMAKE_END_FLAGS} -DCMAKE_PREFIX_PATH=${Torch_DIR}"
fi
if [[ ${WS_DIR} ]]
then
	BUILD_DIR=${WS_DIR}/build/
	# If not global then install to workspace
	if [[ ! ${GLOBAL} ]]
	then
		INSTALL_DIR=${WS_DIR}/install/CoverageControl/
		CMAKE_END_FLAGS="${CMAKE_END_FLAGS} -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}"
	fi
else
	TMP_DIR=$(mktemp -d)
	BUILD_DIR=${TMP_DIR}/build
fi

InstallCoverageControlCore () {
	cmake -S ${DIR}/core -B ${BUILD_DIR}/CoverageControlCore ${CMAKE_END_FLAGS}
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
}

InstallCoverageControlTorch () {
	cmake -S ${DIR}/torch -B ${BUILD_DIR}/CoverageControlTorch ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/CoverageControlTorch -j$(nproc)
	if [ $? -ne 0 ]; then
		echo "CoverageControlTorch build failed"
		exit 1
	fi
	cmake --install ${BUILD_DIR}/CoverageControlTorch
	if [ $? -ne 0 ]; then
		echo "CoverageControlTorch install failed"
		exit 1
	fi

	echo "Successfully built and installed CoverageControlTorch"
}

InstallCoverageControlTests () {
	cmake -S ${DIR}/tests -B ${BUILD_DIR}/CoverageControlTests ${CMAKE_END_FLAGS} -DWITH_TORCH=${WITH_TORCH}
	cmake --build ${BUILD_DIR}/CoverageControlTests -j$(nproc)
	if [ $? -ne 0 ]; then
		echo "CoverageControlTests build failed"
		exit 1
	fi
	cmake --install ${BUILD_DIR}/CoverageControlTests
	if [ $? -ne 0 ]; then
		echo "CoverageControlTests install failed"
		exit 1
	fi

	echo "Successfully built and installed CoverageControlTests"
}

InstallCoverageControlMain () {
	cmake -S ${DIR}/main -B ${BUILD_DIR}/CoverageControlMain ${CMAKE_END_FLAGS} -DWITH_TORCH=${WITH_TORCH}
	cmake --build ${BUILD_DIR}/CoverageControlMain -j$(nproc)
	if [ $? -ne 0 ]; then
		echo "CoverageControlMain build failed"
		exit 1
	fi
	cmake --install ${BUILD_DIR}/CoverageControlMain
	if [ $? -ne 0 ]; then
		echo "CoverageControlMain install failed"
	fi

	echo "Successfully built and installed CoverageControlMain"
}

if [[ ${INSTALL} ]]
then
	InstallCoverageControlCore
	if [[ ${WITH_TORCH} == "ON" ]]
	then
		InstallCoverageControlTorch
		InstallCoverageControlTests
	fi
	InstallCoverageControlMain
fi
