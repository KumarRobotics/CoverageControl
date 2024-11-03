#!/usr/bin/env bash
ORIG_INPUT_PARAMS="$@"
params="$(getopt -o d:hctpg -l directory:,help,clean,torch,python,global,with-cuda,with-deps --name "$(basename "$0")" -- "$@")"

if [ $? -ne 0 ]
then
	print_usage
fi

print_usage() {
	printf "bash $0 [-h|--help] [-d|--directory <workspace directory>] [-p|--python] [--with-cuda] [--with-deps] [-g|--global] [-c|--clean] [-t|--torch]\n"
	printf "Options:\n"
	printf "  -h, --help                             : Prints this help message\n"
	printf "  -d, --directory <workspace directory>  : Builds and installs the package in the specified directory\n"
	printf "  -p, --python                           : Installs the python bindings\n"
	printf "  --with-cuda                            : Builds the package with CUDA support\n"
	printf "  --with-deps                            : Installs the dependencies\n"
	printf "  -g, --global                           : Installs the package globally. Needs sudo permissions\n"
}

# Get directory of script
# https://stackoverflow.com/questions/59895/getting-the-source-directory-of-a-bash-script-from-within
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

WITH_TORCH=0
eval set -- "$params"
unset params

INSTALL=true
while true; do
	case ${1} in
		-h|--help) print_usage; exit 0;;
		-c|--clean) CLEAN=true;INSTALL=false;shift;;
		-t|--torch) WITH_TORCH=ON; shift;;
		-p|--python) WITH_PYTHON=true;shift;;
		-d|--directory) WS_DIR+=("${2}");shift 2;;
		-g|--global) GLOBAL=true;shift;;
		--with-cuda) WITH_CUDA=ON;shift;;
		--with-deps) WITH_DEPS=true;shift;;
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
		INSTALL_DIR=${WS_DIR}/install/
		CMAKE_END_FLAGS="${CMAKE_END_FLAGS} -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}"
	fi
else
	TMP_DIR=$(mktemp -d)
	BUILD_DIR=${TMP_DIR}/build
fi

# Check WITH_CUDA
if [[ ${WITH_CUDA} == "ON" ]]
then
	CMAKE_END_FLAGS="${CMAKE_END_FLAGS} -DWITH_CUDA=ON"
else
	CMAKE_END_FLAGS="${CMAKE_END_FLAGS}"
fi

if [[ ${WITH_TORCH} == "ON" ]]
then
	CMAKE_END_FLAGS="${CMAKE_END_FLAGS} -DCMAKE_PREFIX_PATH=${Torch_DIR}"
fi

InstallCoverageControl () {
  export CGAL_DISABLE_GMP=ON
	export CGAL_DISABLE_GMP=1; cmake -S ${DIR}/core -B ${BUILD_DIR}/CoverageControl ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/CoverageControl -j$(nproc)
	if [ $? -ne 0 ]; then
		echo "CoverageControl build failed"
		exit 1
	fi
	cmake --install ${BUILD_DIR}/CoverageControl
	if [ $? -ne 0 ]; then
		echo "CoverageControl install failed"
	fi

	echo "Successfully built and installed CoverageControl"
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
	echo "Installing CoverageControl"
	InstallCoverageControl
	if [[ ${WITH_TORCH} == "ON" ]]
	then
		echo "Installing CoverageControlTorch"
		InstallCoverageControlTorch
	fi
	# echo "Installing CoverageControlTests"
	# InstallCoverageControlTests
	echo "Installing CoverageControlMain"
	InstallCoverageControlMain
fi
