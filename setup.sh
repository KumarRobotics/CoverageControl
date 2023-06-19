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

if [[ ${INSTALL} ]]
then
	bash cppsrc/setup.sh -i
	if [ $? -ne 0 ]; then
		echo "cppsrc build failed"
		exit 1
	fi
	cd cppsrc/core/python_bindings/
	pip install .
	if [ $? -ne 0 ]; then
		echo "pyCoverageControl failed"
		exit 1
	fi
	# cd ../../torch/python_bindings/
	# pip install .
	# if [ $? -ne 0 ]; then
	# 	echo "pyCoverageControlTorch failed"
	# 	exit 1
	# fi
	cd ../../../python/learning/
	pip install -e .
	cd ../../..
fi

if [[ ${CLEAN} ]]
then
	bash cppsrc/setup.sh -c
fi
