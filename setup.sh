#!/usr/bin/env bash
ORIG_INPUT_PARAMS="$@"
params="$(getopt -o d:ctpg -l directory:,clean,torch,python,global,with-cuda,with-deps --name "$(basename "$0")" -- "$@")"

if [ $? -ne 0 ]
then
	print_usage
fi

print_usage() {
	printf "bash $0 [-d|--directory <workspace directory>] [-c|--clean] [-t|--torch <build with libtorch>] [-p|--python <install python bindings>] [-g|--global] [--with-cuda <cpu only>][--with-deps <install dependencies>]\n"
}

eval set -- "$params"
unset params

WITH_TORCH=0
INSTALL=true
WITH_DEPS=false
while true; do
	case ${1} in
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

# Get directory of script
# https://stackoverflow.com/questions/59895/getting-the-source-directory-of-a-bash-script-from-within
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

if [[ ${WITH_DEPS} == "true" ]]
then
	echo "Installing dependencies"
	if [[ ${WS_DIR} ]]
	then
		INSTALL_DIR=${WS_DIR}/install/
		bash ${DIR}/setup_utils/install_dependencies.sh -d ${INSTALL_DIR}
	else
		bash ${DIR}/setup_utils/install_dependencies.sh
	fi
	if [ $? -ne 0 ]; then
		echo "deps build failed"
		exit 1
	fi
fi

if [[ ${INSTALL} ]]
then
	bash ${DIR}/cppsrc/setup.sh ${ORIG_INPUT_PARAMS}
	if [ $? -ne 0 ]; then
		echo "cppsrc build failed"
		exit 1
	fi
fi

if [[ ${WITH_PYTHON} ]]
then
	echo "Installing python bindings"
	# pip install --no-build-isolation ${DIR}/cppsrc/core/python_bindings/
	pip install ${DIR}/cppsrc/core/python_bindings/
	if [ $? -ne 0 ]; then
		echo "python bindings build failed"
		exit 1
	fi
	pip install -e ${DIR}/python/
	if [ $? -ne 0 ]; then
		echo "CoverageControlTorch Python package install failed"
		exit 1
	fi
fi

# if clean and WITH_PYTHON, then uninstall
if [[ ${CLEAN} ]] && [[ ${WITH_PYTHON} ]]
then
	echo "Cleaning python bindings"
	pip uninstall -y pyCoverageControl
	pip uninstall -y CoverageControlTorch
fi
# pip install -e ${DIR}/torch/python_bindings/
# if [ $? -ne 0 ]; then
# 	echo "pyCoverageControlTorch failed"
# 	exit 1
# fi
