#!/usr/bin/env bash

TMP_DIR=$(mktemp -d)

params="$(getopt -o d: -l directory:,no-cuda,boost,gmp,mpfr,eigen,cgal,pybind11,yaml-cpp,geographiclib,opencv --name "$(basename "$0")" -- "$@")"

print_usage() {
  printf "bash %s [-d|--directory <specify install directory>] [--no-cuda] [--boost] [--gmp] [--mpfr] [--eigen] [--cgal] [--pybind11] [--yaml-cpp] [--geographiclib] [--opencv]\n" "$(basename "$0")"
}
eval set -- "$params"
unset params

while true; do
	case ${1} in
		-d|--directory) INSTALL_DIR+=("${2}");shift 2;;
		--no-cuda) NOCUDA=true;shift;;
		--boost) BOOST=true;shift;;
		--gmp) GMP=true;shift;;
		--mpfr) MPFR=true;shift;;
		--eigen) EIGEN=true;shift;;
		--cgal) CGAL=true;shift;;
		--pybind11) PYBIND11=true;shift;;
		--yaml-cpp) YAML_CPP=true;shift;;
		--geographiclib) GEOGRAPHICLIB=true;shift;;
		--opencv) OPENCV=true;shift;;
		--) shift;break;;
		*) print_usage
			exit 1 ;;
	esac
done

MAIN_DIR=${TMP_DIR}/main/
BUILD_DIR=${TMP_DIR}/build/

if [ -z "$INSTALL_DIR" ]; then
	CMAKE_END_FLAGS="-DCMAKE_BUILD_TYPE=Release"
else
	CMAKE_END_FLAGS="-DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR"
	CONFIGURE_END_FLAGS="--prefix=$INSTALL_DIR"
	echo "Installing to $INSTALL_DIR"
fi

InstallCGAL () {
	echo "Setting up CGAL"
	wget --tries=4 https://github.com/CGAL/cgal/releases/download/v${CGAL_VERSION}/CGAL-${CGAL_VERSION}-library.tar.xz -P ${MAIN_DIR}/src > /dev/null
  if [ $? -ne 0 ]; then
    echo "Failed to download CGAL"
    exit 1
  fi
	tar -xf ${MAIN_DIR}/src/CGAL-${CGAL_VERSION}-library.tar.xz -C ${MAIN_DIR}/src/ > /dev/null
	cmake -S ${MAIN_DIR}/src/CGAL-${CGAL_VERSION} -B ${BUILD_DIR}/cgal ${CMAKE_END_FLAGS} > /dev/null
	cmake --install ${BUILD_DIR}/cgal > /dev/null
	if [ $? -eq 0 ]; then
		echo "cgal install succeeded"
	else
		echo "cgal install failed"
		exit 1
	fi
}

InstallGeoGraphicLib () {
	echo "Setting up geographiclib"
	wget --tries=4 https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.3.tar.gz -P ${MAIN_DIR}/src
  if [ $? -ne 0 ]; then
    echo "Failed to download geographiclib"
    exit 1
  fi
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
	wget --tries=4 https://github.com/pybind/pybind11/archive/refs/tags/v2.12.0.tar.gz -P ${MAIN_DIR}/src
  if [ $? -ne 0 ]; then
    echo "Failed to download pybind11"
    exit 1
  fi
	tar -xf ${MAIN_DIR}/src/v2.12.0.tar.gz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/pybind11-2.12.0 -B ${BUILD_DIR}/pybind11 -DPYBIND11_TEST=OFF ${CMAKE_END_FLAGS}
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
	wget --tries=4 https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz -P ${MAIN_DIR}/src
  if [ $? -ne 0 ]; then
    echo "Failed to download yaml-cpp"
    exit 1
  fi
	tar -xf ${MAIN_DIR}/src/0.8.0.tar.gz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/yaml-cpp-0.8.0 -B ${BUILD_DIR}/yaml-cpp -DYAML_BUILD_SHARED_LIBS=ON  ${CMAKE_END_FLAGS} -DCMAKE_POSITION_INDEPENDENT_CODE=ON
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
	wget --tries=4 https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/${EIGEN_TAR_NAME}.tar.gz -P ${MAIN_DIR}/src
  if [ $? -ne 0 ]; then
    echo "Failed to download eigen3"
    exit 1
  fi
	tar -xf ${MAIN_DIR}/src/${EIGEN_TAR_NAME}.tar.gz -C ${MAIN_DIR}/src/
	cmake -S ${MAIN_DIR}/src/${EIGEN_TAR_NAME} -B ${BUILD_DIR}/eigen3 ${CMAKE_END_FLAGS} > /dev/null
	cmake --build ${BUILD_DIR}/eigen3 -j$(nproc) > /dev/null
	cmake --install ${BUILD_DIR}/eigen3 > /dev/null
	if [ $? -eq 0 ]; then
		echo "eigen3 install succeeded"
	else
		echo "eigen3 install failed"
		exit 1
	fi
}

InstallTorchVision () {
	echo "Setting up torchvision"
	wget --tries=4 https://github.com/pytorch/vision/archive/refs/tags/v0.15.2.tar.gz -P ${MAIN_DIR}/src
  if [ $? -ne 0 ]; then
    echo "Failed to download torchvision"
    exit 1
  fi
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
	wget --tries=4 -O ${MAIN_DIR}/src/opencv.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.8.0.tar.gz
  if [ $? -ne 0 ]; then
    echo "Failed to download opencv"
    exit 1
  fi
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

InstallGMP () {
	echo "Setting up gmp"
	wget --tries=1 https://gmplib.org/download/gmp/${GMP_TAR_NAME}.tar.xz -P ${MAIN_DIR}/src > /dev/null
  if [ $? -ne 0 ]; then
    wget --tries=4 https://github.com/AgarwalSaurav/gmp-mpfr/releases/download/${GMP_TAR_NAME}/${GMP_TAR_NAME}.tar.xz -P ${MAIN_DIR}/src > /dev/null
  fi
  if [ $? -ne 0 ]; then
    echo "Failed to download gmp"
    exit 1
  fi
	tar -xf ${MAIN_DIR}/src/${GMP_TAR_NAME}.tar.xz -C ${MAIN_DIR}/src/ > /dev/null
	cd ${MAIN_DIR}/src/${GMP_TAR_NAME}
	./configure --enable-cxx --disable-shared --with-pic ${CONFIGURE_END_FLAGS}  > /dev/null
	make -C ${MAIN_DIR}/src/${GMP_TAR_NAME} install -j$(nproc) > /dev/null
	if [ $? -eq 0 ]; then
		echo "gmp install succeeded"
	else
		echo "gmp install failed"
		exit 1
	fi
}

InstallMPFR() {
	echo "Setting up mpfr"
	wget --tries=1 https://www.mpfr.org/mpfr-current/${MPFR_TAR_NAME}.tar.xz -P ${MAIN_DIR}/src > /dev/null
  if [ $? -ne 0 ]; then
    wget --tries=4 https://github.com/AgarwalSaurav/gmp-mpfr/releases/download/${MPFR_TAR_NAME}/${MPFR_TAR_NAME}.tar.xz -P ${MAIN_DIR}/src > /dev/null
  fi
  if [ $? -ne 0 ]; then
    echo "Failed to download mpfr"
    exit 1
  fi
	tar -xf ${MAIN_DIR}/src/${MPFR_TAR_NAME}.tar.xz -C ${MAIN_DIR}/src/
	cd ${MAIN_DIR}/src/${MPFR_TAR_NAME}
	# if ${INSTALL_DIR} is set, then we need to tell where gmp is installed
	if [ -z "$INSTALL_DIR" ]; then
		./configure --disable-shared --with-pic ${CONFIGURE_END_FLAGS}
	else
		./configure --disable-shared --with-pic --with-gmp=${INSTALL_DIR} ${CONFIGURE_END_FLAGS}
	fi
	make -C ${MAIN_DIR}/src/${MPFR_TAR_NAME} install -j$(nproc)
	if [ $? -eq 0 ]; then
		echo "mpfr install succeeded"
	else
		echo "mpfr install failed"
		exit 1
	fi
}

InstallBoost () {
	echo "Setting up boost"
	wget --tries=4 https://boostorg.jfrog.io/artifactory/main/release/${BOOST_VERSION}/source/${BOOST_TAR_NAME}.tar.gz -P ${MAIN_DIR}/src
  if [ $? -ne 0 ]; then
    echo "Failed to download boost"
    exit 1
  fi
	tar -xf ${MAIN_DIR}/src/${BOOST_TAR_NAME}.tar.gz -C ${MAIN_DIR}/src/
	cd ${MAIN_DIR}/src/${BOOST_TAR_NAME}
	if [ -z "$INSTALL_DIR" ]; then
		./bootstrap.sh
	else
		./bootstrap.sh --prefix=${INSTALL_DIR}
	fi
	./b2 cxxflags="-fPIC" cflags="-fPIC" -a link=static install -j$(nproc)
	if [ $? -eq 0 ]; then
		echo "boost install succeeded"
	else
		echo "boost install failed"
		exit 1
	fi
}

BOOST_VERSION=1.86.0
BOOST_TAR_NAME=$(echo boost_${BOOST_VERSION} | tr . _)
GMP_VERSION=6.3.0
GMP_TAR_NAME=$(echo gmp-${GMP_VERSION})
MPFR_VERSION=4.2.1
MPFR_TAR_NAME=$(echo mpfr-${MPFR_VERSION})
EIGEN_VERSION=3.4.0
EIGEN_TAR_NAME=$(echo eigen-${EIGEN_VERSION})
CGAL_VERSION=6.0.1
CGAL_TAR_NAME=$(echo CGAL-${CGAL_VERSION})

if [ -n "$BOOST" ]; then
	InstallBoost
fi

if [ -n "$GMP" ]; then
	InstallGMP
fi

if [ -n "$MPFR" ]; then
	InstallMPFR
fi

if [ -n "$EIGEN" ]; then
	InstallEigen3
fi

if [ -n "$CGAL" ]; then
	InstallCGAL
fi

if [ -n "$PYBIND11" ]; then
	InstallPybind11
fi

if [ -n "$YAML_CPP" ]; then
	InstallYamlCPP
fi

if [ -n "$GEOGRAPHICLIB" ]; then
	InstallGeoGraphicLib
fi

if [ -n "$OPENCV" ]; then
	InstallOpenCV
fi


# InstallGMP
# InstallMPFR
# InstallEigen3
# InstallCGAL
# InstallPybind11
# InstallYamlCPP
# InstallGeoGraphicLib
# InstallOpenCV
# InstallTorchVision
# InstallTorchSparse
# InstallTorchScatter
#
rm -rf ${TMP_DIR}
