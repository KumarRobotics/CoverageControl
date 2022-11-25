#!/bin/bash

COVERAGE_CONTROL_INSTALL_PATH=${HOME}/CoverageControl/
mkdir -p build
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=${COVERAGE_CONTROL_INSTALL_PATH}
cmake --build build/
cmake --install build/
