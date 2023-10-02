TMP_DIR=`mktemp -d`

INSTALL_DIR=${1}
mkdir -p ${INSTALL_DIR}
wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu118.zip -O ${TMP_DIR}/libtorch.zip
unzip ${TMP_DIR}/libtorch.zip -d ${INSTALL_DIR}/
# cp -r ${TMP_DIR}/libtorch/* ${1}/.
rm -r ${TMP_DIR}
