#!/bin/bash
print_usage() {
	printf "bash $0 [-c (for clean)] [-i (for install)]\n"
}

WITH_TORCH=0
WITH_PYTHON=true
while getopts 'ictp' flag; do
	case "${flag}" in
		i) INSTALL=true;;
		c) CLEAN=true;;
		t) WITH_TORCH=true;;
		p) WITH_PYTHON=true;;
		*) print_usage
			exit 1 ;;
	esac
done

bash cppsrc/setup.sh $@
if [ $? -ne 0 ]; then
	echo "cppsrc build failed"
	exit 1
fi
if [[ ${INSTALL} ]]
then
	echo "Installing python bindings"
	cd cppsrc/core/python_bindings/
	pip install .
	if [ $? -ne 0 ]; then
		echo "python bindings build failed"
		exit 1
	fi
	cd ../../../python/learning/
	pip install -e .
	cd ../../..
	if [ $? -ne 0 ]; then
		echo "CoverageControlTorch Python package install failed"
		exit 1
	fi
fi
# cd ../../torch/python_bindings/
# pip install .
# if [ $? -ne 0 ]; then
# 	echo "pyCoverageControlTorch failed"
# 	exit 1
# fi
