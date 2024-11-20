#!/usr/bin/env bash
print_usage() {
	printf "bash %s [-h|--help] [-d|--directory <workspace directory>] [-p|--python] [--with-cuda] [--with-deps] [-g|--global] [-c|--clean] [-t|--torch]\n" "$0"
	printf "Options:\n"
	printf "  -h, --help                             : Prints this help message\n"
	printf "  -d, --directory <workspace directory>  : Builds and installs the package in the specified directory\n"
	printf "  -p, --python                           : Installs the python bindings\n"
	printf "  --with-cuda                            : Builds the package with CUDA support\n"
	printf "  --with-deps                            : Installs the dependencies\n"
	printf "  -g, --global                           : Installs the package globally. Needs sudo permissions\n"
}

ORIG_INPUT_PARAMS="$@"

params="$(getopt -o d:hctpg -l directory:,help,clean,torch,python,global,with-cuda,with-deps,pip-path: --name "$(basename "$0")" -- "$@")"

if [ $? -ne 0 ]
then
	print_usage
fi

eval set -- "$params"
echo "Params: $params"
unset params

WITH_TORCH=0
INSTALL=true
WITH_DEPS=false
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
		--pip-path) PIP_PATH+=("${2}");shift 2;;
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
		bash ${DIR}/utils/setup/install_dependencies.sh -d ${INSTALL_DIR} --eigen --cgal
	else
		bash ${DIR}/utils/setup/install_dependencies.sh --eigen --cgal
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
	echo "pip_path: $(which pip)"

	# pip install --no-build-isolation ${DIR}/cppsrc/core/python_bindings/
	# pip install ${DIR}/cppsrc/core/python_bindings/
  pip install .
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
