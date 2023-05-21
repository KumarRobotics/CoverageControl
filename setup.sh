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
	cd python/core
	pip install .
	cd ../torch
	pip install .
	cd ../..
fi

if [[ ${CLEAN} ]]
then
	echo "Cleaning build and install directories"
	CleanBuild
fi
