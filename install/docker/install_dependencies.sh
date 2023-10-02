#!/usr/bin/env bash

TMP_DIR=`mktemp -d`
MAIN_DIR=${TMP_DIR}/main/
BUILD_DIR=${TMP_DIR}/build/

if [ -z "$1" ]; then
	CMAKE_END_FLAGS="-DCMAKE_BUILD_TYPE=Release"
else
	CMAKE_END_FLAGS="-DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$1"
fi

InstallCGAL () {
	echo "Setting up CGAL"
	wget https://github.com/CGAL/cgal/releases/download/v5.6/CGAL-5.6-library.tar.xz -P ${MAIN_DIR}/src
	tar -xf ${MAIN_DIR}/src/CGAL-5.6-library.tar.xz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/CGAL-5.6 -B ${BUILD_DIR}/cgal ${CMAKE_END_FLAGS}
	cmake --install ${BUILD_DIR}/cgal
	if [ $? -eq 0 ]; then
		echo "cgal install succeeded"
	else
		echo "cgal install failed"
		exit 1
	fi
}

InstallGeoGraphicLib () {
	echo "Setting up geographiclib"
	wget https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.3.tar.gz -P ${MAIN_DIR}/src
	tar -xf ${MAIN_DIR}/src/v2.3.tar.gz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/geographiclib-2.3 -B ${BUILD_DIR}/geographiclib ${CMAKE_END_FLAGS} -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
	cmake --build ${BUILD_DIR}/geographiclib -j$(nproc)
	cmake --install ${BUILD_DIR}/geographiclib
	if [ $? -eq 0 ]; then
		echo "geographiclib install succeeded"
	else
		echo "geographiclib install failed"
		exit 1
	fi
}

InstallPybind11 () {
	echo "Setting up pybind11"
	wget https://github.com/pybind/pybind11/archive/refs/tags/v2.11.1.tar.gz -P ${MAIN_DIR}/src
	tar -xf ${MAIN_DIR}/src/v2.11.1.tar.gz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/pybind11-2.11.1 -B ${BUILD_DIR}/pybind11 -DPYBIND11_TEST=OFF ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/pybind11 -j$(nproc)
	cmake --install ${BUILD_DIR}/pybind11
	if [ $? -eq 0 ]; then
		echo "pybind11 install succeeded"
	else
		echo "pybind11 install failed"
		exit 1
	fi
}

InstallYamlCPP () {
	echo "Setting up yaml-cpp"
	wget https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz -P ${MAIN_DIR}/src
	tar -xf ${MAIN_DIR}/src/0.8.0.tar.gz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/yaml-cpp-0.8.0 -B ${BUILD_DIR}/yaml-cpp ${CMAKE_END_FLAGS} -DYAML_BUILD_SHARED_LIBS=ON
	cmake --build ${BUILD_DIR}/yaml-cpp -j$(nproc)
	if [ $? -ne 0 ]; then
		echo "YAML build failed"
	fi
	cmake --install ${BUILD_DIR}/yaml-cpp
	if [ $? -eq 0 ]; then
		echo "yaml-cpp install succeeded"
	else
		echo "yaml-cpp install failed"
		exit 1
	fi
}

InstallEigen3 () {
	echo "Setting up eigen3"
	wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz -P ${MAIN_DIR}/src
	tar -xf ${MAIN_DIR}/src/eigen-3.4.0.tar.gz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/eigen-3.4.0 -B ${BUILD_DIR}/eigen3 ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/eigen3
	cmake --install ${BUILD_DIR}/eigen3
	if [ $? -eq 0 ]; then
		echo "eigen3 install succeeded"
	else
		echo "eigen3 install failed"
		exit 1
	fi
}

InstallTorchVision () {
	echo "Setting up torchvision"
	wget https://github.com/pytorch/vision/archive/refs/tags/v0.15.2.tar.gz -P ${MAIN_DIR}/src
	tar -xf ${MAIN_DIR}/src/v0.15.2.tar.gz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/vision-0.15.2 -B ${BUILD_DIR}/torchvision -DWITH_CUDA=ON -DUSE_PYTHON=ON -DCMAKE_INSTALL_PREFIX=${Torch_ROOT} ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/torchvision -j$(nproc)
	cmake --install ${BUILD_DIR}/torchvision
	if [ $? -eq 0 ]; then
		echo "torchvision install succeeded"
	else
		echo "torchvision install failed"
		exit 1
	fi
}

InstallTorchScatter () {
	echo "Setting up torchscatter"
	git clone --recurse-submodules https://github.com/rusty1s/pytorch_scatter.git ${MAIN_DIR}/src/pytorch_scatter
	cmake -S ${MAIN_DIR}/src/pytorch_scatter -B ${BUILD_DIR}/torchscatter -DWITH_CUDA=ON -DWITH_PYTHON=ON -DCMAKE_INSTALL_PREFIX=${Torch_ROOT} ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/torchscatter -j$(nproc)
	cmake --install ${BUILD_DIR}/torchscatter
	if [ $? -eq 0 ]; then
		echo "torchscatter install succeeded"
	else
		echo "torchscatter install failed"
		exit 1
	fi
}

InstallTorchSparse () {
	echo "Setting up torchsparse"
	git clone --recurse-submodules https://github.com/rusty1s/pytorch_sparse.git ${MAIN_DIR}/src/pytorch_sparse
	cmake -S ${MAIN_DIR}/src/pytorch_sparse -B ${BUILD_DIR}/torchsparse -DWITH_CUDA=ON -DWITH_PYTHON=ON -DCMAKE_INSTALL_PREFIX=${Torch_ROOT} ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/torchsparse -j$(nproc)
	cmake --install ${BUILD_DIR}/torchsparse
	if [ $? -eq 0 ]; then
		echo "torchsparse install succeeded"
	else
		echo "torchsparse install failed"
		exit 1
	fi
}

InstallOpenCV () {
	echo "Setting up opencv"
	wget -O ${MAIN_DIR}/src/opencv.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.8.0.tar.gz
	wget -O ${MAIN_DIR}/src/opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.8.0.zip
	tar -xf ${MAIN_DIR}/src/opencv.tar.gz -C ${MAIN_DIR}/src/
	unzip ${MAIN_DIR}/src/opencv_contrib.zip -d ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/opencv-4.8.0 -B ${BUILD_DIR}/opencv -DWITH_CUDA=ON -DWITH_CUBLAS=ON -DWITH_CUDNN=ON -DWITH_FFMPEG=ON -DWITH_EIGEN=ON -DWITH_OPENMP=ON -DWITH_JPEG=ON -DWITH_PNG=ON -DWITH_TIFF=ON -DWITH_OPENJPEG=ON -DOPENCV_EXTRA_MODULES_PATH=${MAIN_DIR}/src/opencv_contrib-4.8.0/modules ${CMAKE_END_FLAGS}
	cmake --build ${BUILD_DIR}/opencv -j$(nproc)
	cmake --install ${BUILD_DIR}/opencv
	if [ $? -eq 0 ]; then
		echo "opencv install succeeded"
	else
		echo "opencv install failed"
		exit 1
	fi
}

InstallEigen3
# InstallPybind11
InstallCGAL
InstallYamlCPP
InstallGeoGraphicLib
# InstallOpenCV
# InstallTorchVision
# InstallTorchSparse
# InstallTorchScatter
#
rm -rf ${TMP_DIR}
