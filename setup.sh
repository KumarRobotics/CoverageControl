#!/usr/bin/env bash
print_usage() {
	printf "bash $0 [-c <for clean>] [-i <for install>] [-t <for torch>] [-p <for python>] [-d <workspace_dir>]\n"
}

# Get directory of script
# https://stackoverflow.com/questions/59895/getting-the-source-directory-of-a-bash-script-from-within
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

WITH_TORCH=0
while getopts 'd:ictpg' flag; do
	case "${flag}" in
		i) INSTALL=true;;
		c) CLEAN=true;;
		t) WITH_TORCH=true;;
		p) WITH_PYTHON=true;;
		d) WS_DIR=${OPTARG};;
		g) GLOBAL=true;;
		*) print_usage
			exit 1 ;;
	esac
done

if [[ ${INSTALL} ]]
then
	bash ${DIR}/cppsrc/setup.sh $@
	if [ $? -ne 0 ]; then
		echo "cppsrc build failed"
		exit 1
	fi
fi

if [[ ${WITH_PYTHON} ]]
then
	echo "Installing python bindings"
	pip install ${DIR}/cppsrc/core/python_bindings/
	if [ $? -ne 0 ]; then
		echo "python bindings build failed"
		exit 1
	fi
	pip install -e ${DIR}/python/learning/
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
