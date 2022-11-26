#!/bin/bash
WORKSPACE_DIR=${HOME}/CoverageControl_ws
BUILD_DIR=${WORKSPACE_DIR}/build
INSTALL_DIR=${WORKSPACE_DIR}/install

mkdir -p ${WORKSPACE_DIR}/src
cd ${WORKSPACE_DIR}/src
git clone --recursive git@github.com:AgarwalSaurav/CoverageControl.git
cd ..

cmake -S ${WORKSPACE_DIR}/src/CoverageControl/extern/pybind11 -B ${BUILD_DIR}/pybind11 -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
cmake --install ${BUILD_DIR}/pybind11

cmake -S ${WORKSPACE_DIR}/src/CoverageControl -B ${BUILD_DIR}/CoverageControl -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release
cmake --build ${BUILD_DIR}/CoverageControl
cmake --install ${BUILD_DIR}/CoverageControl
