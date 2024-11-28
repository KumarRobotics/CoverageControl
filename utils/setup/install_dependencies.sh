#!/usr/bin/env bash

set -euo pipefail

RED='\033[0;31m'    # Red
GREEN='\033[0;32m'  # Green
YELLOW='\033[0;33m' # Yellow
NC='\033[0m'        # No Color

error_exit() {
  echo -e "${RED}Error: $1${NC}" >&2
  exit 1
}

info_message() {
  echo -e "${GREEN}$1${NC}"
}

warning_message() {
  echo -e "${YELLOW}$1${NC}"
}

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

print_usage() {
  cat <<EOF
Usage: bash $(basename "$0") [OPTIONS]

Options:
  -d, --directory <install directory>  Specify the installation directory.
      --boost                          Install Boost library.
      --gmp                            Install GMP library.
      --mpfr                           Install MPFR library.
      --eigen                          Install Eigen library.
      --cgal                           Install CGAL library.
      --pybind11                       Install PyBind11 library.
      --yaml-cpp                       Install YAML-CPP library.
      --geographiclib                  Install GeographicLib library.
      --opencv                         Install OpenCV library.

  -h, --help                           Show this help message and exit.

Examples:
  bash $(basename "$0") --boost --gmp
  bash $(basename "$0") -d /usr/local --opencv

Note:
  - You can specify multiple libraries to install.
EOF
}

for cmd in wget cmake make tar; do
  if ! command_exists "$cmd"; then
    error_exit "'$cmd' command is not found. Please install it before running this script."
  fi
done

# Create a temporary directory and ensure it is cleaned up on exit
TMP_DIR=$(mktemp -d)
trap 'rm -rf "$TMP_DIR"' EXIT

MAIN_DIR="${TMP_DIR}/main"
BUILD_DIR="${TMP_DIR}/build"
mkdir -p "$MAIN_DIR/src"
mkdir -p "$BUILD_DIR"

if [[ $# -eq 0 ]]; then
  print_usage
  exit 1
fi

INSTALL_DIR=""
BOOST=false
GMP=false
MPFR=false
EIGEN=false
CGAL=false
PYBIND11=false
YAML_CPP=false
GEOGRAPHICLIB=false
OPENCV=false

SHORT_OPTS="d:h"
LONG_OPTS="directory:,boost,gmp,mpfr,eigen,cgal,pybind11,yaml-cpp,geographiclib,opencv,help"

# Parse options using getopt
PARSED_PARAMS=$(getopt -o "$SHORT_OPTS" -l "$LONG_OPTS" --name "$(basename "$0")" -- "$@") || {
  error_exit "Failed to parse arguments."
}

eval set -- "$PARSED_PARAMS"

while true; do
  case "$1" in
    -d|--directory)
      INSTALL_DIR="$2"
      shift 2
      ;;
    --boost)
      BOOST=true
      shift
      ;;
    --gmp)
      GMP=true
      shift
      ;;
    --mpfr)
      MPFR=true
      shift
      ;;
    --eigen)
      EIGEN=true
      shift
      ;;
    --cgal)
      CGAL=true
      shift
      ;;
    --pybind11)
      PYBIND11=true
      shift
      ;;
    --yaml-cpp)
      YAML_CPP=true
      shift
      ;;
    --geographiclib)
      GEOGRAPHICLIB=true
      shift
      ;;
    --opencv)
      OPENCV=true
      shift
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    *)
      error_exit "Unknown option: $1"
      ;;
  esac
done

# Set CMake and configure flags
if [[ -z "$INSTALL_DIR" ]]; then
  CMAKE_END_FLAGS="-DCMAKE_BUILD_TYPE=Release"
  CONFIGURE_END_FLAGS=""
else
  CMAKE_END_FLAGS="-DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR"
  CONFIGURE_END_FLAGS="--prefix=$INSTALL_DIR"
  info_message "Installing to $INSTALL_DIR"
fi

# ----------------------------
# Library Versions
# ----------------------------

BOOST_VERSION="1.86.0"
BOOST_TAR_NAME="boost_$(echo $BOOST_VERSION | tr '.' '_')"
GMP_VERSION="6.3.0"
GMP_TAR_NAME="gmp-${GMP_VERSION}"
MPFR_VERSION="4.2.1"
MPFR_TAR_NAME="mpfr-${MPFR_VERSION}"
EIGEN_VERSION="3.4.0"
EIGEN_TAR_NAME="eigen-${EIGEN_VERSION}"
CGAL_VERSION="6.0.1"
CGAL_TAR_NAME="CGAL-${CGAL_VERSION}"
OPENCV_VERSION="4.8.0"

InstallBoost() {
  info_message "Setting up Boost"
  wget -q --tries=4 "https://boostorg.jfrog.io/artifactory/main/release/${BOOST_VERSION}/source/${BOOST_TAR_NAME}.tar.gz" -P "${MAIN_DIR}/src" || error_exit "Failed to download Boost"
  tar -xf "${MAIN_DIR}/src/${BOOST_TAR_NAME}.tar.gz" -C "${MAIN_DIR}/src"
  cd "${MAIN_DIR}/src/${BOOST_TAR_NAME}"
  if [[ -z "$INSTALL_DIR" ]]; then
    ./bootstrap.sh
  else
    ./bootstrap.sh --prefix="${INSTALL_DIR}"
  fi
  ./b2 cxxflags="-fPIC" cflags="-fPIC" -a link=static install -j"$(nproc)" || error_exit "Boost install failed"
  info_message "Boost install succeeded"
}

InstallGMP() {
  info_message "Setting up GMP"
  wget -q --tries=1 "https://gmplib.org/download/gmp/${GMP_TAR_NAME}.tar.xz" -P "${MAIN_DIR}/src" || \
  wget -q --tries=4 "https://github.com/AgarwalSaurav/gmp-mpfr/releases/download/${GMP_TAR_NAME}/${GMP_TAR_NAME}.tar.xz" -P "${MAIN_DIR}/src" || \
  error_exit "Failed to download GMP"
  tar -xf "${MAIN_DIR}/src/${GMP_TAR_NAME}.tar.xz" -C "${MAIN_DIR}/src"
  cd "${MAIN_DIR}/src/${GMP_TAR_NAME}"
  ./configure --enable-cxx --disable-shared --with-pic ${CONFIGURE_END_FLAGS}
  make -j"$(nproc)" install || error_exit "GMP install failed"
  info_message "GMP install succeeded"
}

InstallMPFR() {
  info_message "Setting up MPFR"
  wget -q --tries=1 "https://www.mpfr.org/mpfr-current/${MPFR_TAR_NAME}.tar.xz" -P "${MAIN_DIR}/src" || \
  wget -q --tries=4 "https://github.com/AgarwalSaurav/gmp-mpfr/releases/download/${MPFR_TAR_NAME}/${MPFR_TAR_NAME}.tar.xz" -P "${MAIN_DIR}/src" || \
  error_exit "Failed to download MPFR"
  tar -xf "${MAIN_DIR}/src/${MPFR_TAR_NAME}.tar.xz" -C "${MAIN_DIR}/src"
  cd "${MAIN_DIR}/src/${MPFR_TAR_NAME}"
  if [[ -z "$INSTALL_DIR" ]]; then
    ./configure --disable-shared --with-pic ${CONFIGURE_END_FLAGS}
  else
    ./configure --disable-shared --with-pic --with-gmp="${INSTALL_DIR}" ${CONFIGURE_END_FLAGS}
  fi
  make -j"$(nproc)" install || error_exit "MPFR install failed"
  info_message "MPFR install succeeded"
}

InstallEigen3() {
  info_message "Setting up Eigen3"
  wget -q --tries=4 "https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/${EIGEN_TAR_NAME}.tar.gz" -P "${MAIN_DIR}/src" || error_exit "Failed to download Eigen3"
  tar -xf "${MAIN_DIR}/src/${EIGEN_TAR_NAME}.tar.gz" -C "${MAIN_DIR}/src"
  cmake -S "${MAIN_DIR}/src/${EIGEN_TAR_NAME}" -B "${BUILD_DIR}/eigen3" ${CMAKE_END_FLAGS}
  cmake --build "${BUILD_DIR}/eigen3" -j"$(nproc)"
  cmake --install "${BUILD_DIR}/eigen3" || error_exit "Eigen3 install failed"
  info_message "Eigen3 install succeeded"
}

InstallCGAL() {
  info_message "Setting up CGAL"
  wget -q --tries=4 "https://github.com/CGAL/cgal/releases/download/v${CGAL_VERSION}/${CGAL_TAR_NAME}-library.tar.xz" -P "${MAIN_DIR}/src" || error_exit "Failed to download CGAL"
  tar -xf "${MAIN_DIR}/src/${CGAL_TAR_NAME}-library.tar.xz" -C "${MAIN_DIR}/src"
  cmake -S "${MAIN_DIR}/src/${CGAL_TAR_NAME}" -B "${BUILD_DIR}/cgal" ${CMAKE_END_FLAGS}
  cmake --build "${BUILD_DIR}/cgal" -j"$(nproc)"
  cmake --install "${BUILD_DIR}/cgal" || error_exit "CGAL install failed"
  info_message "CGAL install succeeded"
}

InstallPybind11() {
  info_message "Setting up Pybind11"
  wget -q --tries=4 "https://github.com/pybind/pybind11/archive/refs/tags/v2.12.0.tar.gz" -P "${MAIN_DIR}/src" || error_exit "Failed to download Pybind11"
  tar -xf "${MAIN_DIR}/src/v2.12.0.tar.gz" -C "${MAIN_DIR}/src"
  cmake -S "${MAIN_DIR}/src/pybind11-2.12.0" -B "${BUILD_DIR}/pybind11" -DPYBIND11_TEST=OFF ${CMAKE_END_FLAGS}
  cmake --build "${BUILD_DIR}/pybind11" -j"$(nproc)"
  cmake --install "${BUILD_DIR}/pybind11" || error_exit "Pybind11 install failed"
  info_message "Pybind11 install succeeded"
}

InstallYamlCPP() {
  info_message "Setting up yaml-cpp"
  wget -q --tries=4 "https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz" -P "${MAIN_DIR}/src" || error_exit "Failed to download yaml-cpp"
  tar -xf "${MAIN_DIR}/src/0.8.0.tar.gz" -C "${MAIN_DIR}/src"
  cmake -S "${MAIN_DIR}/src/yaml-cpp-0.8.0" -B "${BUILD_DIR}/yaml-cpp" -DYAML_BUILD_SHARED_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON ${CMAKE_END_FLAGS}
  cmake --build "${BUILD_DIR}/yaml-cpp" -j"$(nproc)"
  cmake --install "${BUILD_DIR}/yaml-cpp" || error_exit "yaml-cpp install failed"
  info_message "yaml-cpp install succeeded"
}

InstallGeographicLib() {
  info_message "Setting up GeographicLib"
  wget -q --tries=4 "https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.3.tar.gz" -P "${MAIN_DIR}/src" || error_exit "Failed to download GeographicLib"
  tar -xf "${MAIN_DIR}/src/v2.3.tar.gz" -C "${MAIN_DIR}/src"
  cmake -S "${MAIN_DIR}/src/geographiclib-2.3" -B "${BUILD_DIR}/geographiclib" -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON ${CMAKE_END_FLAGS}
  cmake --build "${BUILD_DIR}/geographiclib" -j"$(nproc)"
  cmake --install "${BUILD_DIR}/geographiclib" || error_exit "GeographicLib install failed"
  info_message "GeographicLib install succeeded"
}

InstallOpenCV() {
  info_message "Setting up OpenCV"
  wget -q --tries=4 -O "${MAIN_DIR}/src/opencv.tar.gz" "https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz" || error_exit "Failed to download OpenCV"
  wget -q --tries=4 -O "${MAIN_DIR}/src/opencv_contrib.tar.gz" "https://github.com/opencv/opencv_contrib/archive/refs/tags/${OPENCV_VERSION}.tar.gz" || error_exit "Failed to download OpenCV contrib modules"
  tar -xf "${MAIN_DIR}/src/opencv.tar.gz" -C "${MAIN_DIR}/src"
  tar -xf "${MAIN_DIR}/src/opencv_contrib.tar.gz" -C "${MAIN_DIR}/src"
  cmake -S "${MAIN_DIR}/src/opencv-${OPENCV_VERSION}" -B "${BUILD_DIR}/opencv" \
    -DWITH_CUDA=ON \
    -DWITH_CUBLAS=ON \
    -DWITH_CUDNN=ON \
    -DWITH_FFMPEG=ON \
    -DWITH_EIGEN=ON \
    -DWITH_OPENMP=ON \
    -DWITH_JPEG=ON \
    -DWITH_PNG=ON \
    -DWITH_TIFF=ON \
    -DWITH_OPENJPEG=ON \
    -DOPENCV_EXTRA_MODULES_PATH="${MAIN_DIR}/src/opencv_contrib-${OPENCV_VERSION}/modules" \
    ${CMAKE_END_FLAGS}
  cmake --build "${BUILD_DIR}/opencv" -j"$(nproc)"
  cmake --install "${BUILD_DIR}/opencv" || error_exit "OpenCV install failed"
  info_message "OpenCV install succeeded"
}

if [[ "$BOOST" == true ]]; then
  InstallBoost
fi

if [[ "$GMP" == true ]]; then
  InstallGMP
fi

if [[ "$MPFR" == true ]]; then
  InstallMPFR
fi

if [[ "$EIGEN" == true ]]; then
  InstallEigen3
fi

if [[ "$CGAL" == true ]]; then
  InstallCGAL
fi

if [[ "$PYBIND11" == true ]]; then
  InstallPybind11
fi

if [[ "$YAML_CPP" == true ]]; then
  InstallYamlCPP
fi

if [[ "$GEOGRAPHICLIB" == true ]]; then
  InstallGeographicLib
fi

if [[ "$OPENCV" == true ]]; then
  InstallOpenCV
fi

info_message "Installation completed successfully."
