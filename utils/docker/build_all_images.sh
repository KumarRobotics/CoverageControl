set -x
print_usage() {
		echo "Usage: ./build_all_images.sh <username/repo_name>"
		echo "Example: ./build_all_images.sh johndoe/repo_name"
		echo "Example: ./build_all_images.sh ghcr.io/johndoe/repo_name"
		exit 1
}

if [ "$#" -ne 1 ]; then
    print_usage
fi

build_image() {
    echo "Building image $2"
    TAG_NAME=$2
    ALL_BUILD_ARGS="--no-cache --build-arg CUDA_VERSION=${CUDA_VERSION} --build-arg PYTHON_VERSION=${PYTHON_VERSION} --build-arg PYTORCH_VERSION=${PYTORCH_VERSION} --build-arg IMAGE_TYPE=${IMAGE_TYPE} --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} --build-arg ROS_DISTRO=${ROS_DISTRO}"
    DOCKER_BUILDKIT=1 docker buildx build --push ${ALL_BUILD_ARGS} $3 -t ${1}:${TAG_NAME} .
    if [ $? -ne 0 ]; then
        echo "Failed to build image $2"
        exit 1
    fi
}

CUDA_VERSION="12.4.1"
PYTHON_VERSION="3.10"
PYTORCH_VERSION="2.5.1"
IMAGE_TYPE="cuda"
UBUNTU_VERSION="22.04"
ROS_DISTRO="humble"

TAG_NAME=jammy-torch${PYTORCH_VERSION}-cuda${CUDA_VERSION}
build_image $1 $TAG_NAME "--target base"

TAG_NAME=jammy-torch${PYTORCH_VERSION}-cuda${CUDA_VERSION}-humble
build_image $1 $TAG_NAME "--target ros2"

IMAGE_TYPE="cpu"
TAG_NAME=jammy-torch${PYTORCH_VERSION}-humble
build_image $1 $TAG_NAME "--target ros2"

TAG_NAME=jammy-torch${PYTORCH_VERSION}
build_image $1 $TAG_NAME "--target base"

TAG_NAME=arm64-jammy-torch${PYTORCH_VERSION}-humble
build_image $1 $TAG_NAME "--platform linux/arm64 --target ros2"

CUDA_VERSION="12.6.2"
PYTHON_VERSION="3.12"
PYTORCH_VERSION="2.5.1"
IMAGE_TYPE="cuda"
UBUNTU_VERSION="24.04"
ROS_DISTRO="jazzy"

TAG_NAME=noble-torch${PYTORCH_VERSION}-cuda${CUDA_VERSION}
build_image $1 $TAG_NAME "--target base"

TAG_NAME=noble-torch${PYTORCH_VERSION}-cuda${CUDA_VERSION}-jazzy
build_image $1 $TAG_NAME "--target ros2"

IMAGE_TYPE="cpu"
TAG_NAME=noble-torch${PYTORCH_VERSION}-jazzy
build_image $1 $TAG_NAME "--target ros2"

TAG_NAME=noble-torch${PYTORCH_VERSION}
build_image $1 $TAG_NAME "--target base -t ${1}:latest"

TAG_NAME=arm64-noble-torch${PYTORCH_VERSION}-jazzy
build_image $1 $TAG_NAME "--platform linux/arm64 --target ros2"
