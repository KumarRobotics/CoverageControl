#!/usr/bin/env bash
params="$(getopt -o d:ictpg -l directory:,install,clean,torch,python,global,no-cuda,no-deps --name "$(basename "$0")" -- "$@")"

if [ $? -ne 0 ]
then
    print_usage
fi

print_usage() {
	printf "bash $0 [-d|--directory <workspace directory>] [-i|--install] [-c|--clean] [-t|--torch <build with libtorch>] [-p|--python <install python bindings>] [-g|--global] [--no-cuda <cpu only>] [--no-deps <do not install dependencies>]\n"
}

# Get directory of script
# https://stackoverflow.com/questions/59895/getting-the-source-directory-of-a-bash-script-from-within
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

WITH_TORCH=0
eval set -- "$params"
unset params

while true; do
	case ${1} in
		-i|--install) INSTALL=true;shift;;
		-c|--clean) CLEAN=true;shift;;
		-t|--torch) WITH_TORCH=ON; shift;;
		-p|--python) WITH_PYTHON=true;shift;;
		-d|--directory) WS_DIR+=("${2}");shift 2;;
		-g|--global) GLOBAL=true;shift;;
		--no-cuda) NOCUDA=true;shift;;
		--no-deps) WITH_DEPS=false;shift;;
		--) shift;break;;
		*) print_usage
			exit 1 ;;
	esac
done

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

if [[ ${NOCUDA} ]]
then
	CMAKE_END_FLAGS="${CMAKE_END_FLAGS} -DWITH_CUDA=OFF"
fi
if [[ ${WITH_TORCH} == "ON" ]]
then
	CMAKE_END_FLAGS="${CMAKE_END_FLAGS} -DCMAKE_PREFIX_PATH=${Torch_DIR}"
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
	echo "Installing CoverageControlCore"
	InstallCoverageControlCore
	if [[ ${WITH_TORCH} == "ON" ]]
	then
		echo "Installing CoverageControlTorch"
		InstallCoverageControlTorch
		echo "Installing CoverageControlTests"
		InstallCoverageControlTests
	fi
	echo "Installing CoverageControlMain"
	InstallCoverageControlMain
fi
