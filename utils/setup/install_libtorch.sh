#!/usr/bin/env bash

TMP_DIR=`mktemp -d`
params="$(getopt -o d: -l directory:,no-cuda --name "$(basename "$0")" -- "$@")"
if [ $? -ne 0 ]
then
    print_usage
fi

print_usage() {
	printf "bash $0 [-d|--directory <specify install directory>] [--no-cuda <to install cpu version>]\n"
}
eval set -- "$params"
unset params

while true; do
	case ${1} in
		-d|--directory) INSTALL_DIR+=("${2}");shift 2;;
		--no-cuda) NOCUDA=true;shift;;
		--) shift;break;;
		*) print_usage
			exit 1 ;;
	esac
done

mkdir -p ${INSTALL_DIR}
if [[ ${NOCUDA} ]]
then
	wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.5.1%2Bcpu.zip -O ${TMP_DIR}/libtorch.zip

else
	wget https://download.pytorch.org/libtorch/cu124/libtorch-cxx11-abi-shared-with-deps-2.5.1%2Bcu124.zip -O ${TMP_DIR}/libtorch.zip
fi
unzip ${TMP_DIR}/libtorch.zip -d ${INSTALL_DIR}/
# cp -r ${TMP_DIR}/libtorch/* ${1}/.
rm -r ${TMP_DIR}
