print_usage() {
		echo "Usage: ./build_all_images.sh <username/repo_name>"
		echo "Example: ./build_all_images.sh johndoe/repo_name"
		echo "Example: ./build_all_images.sh ghcr.io/johndoe/repo_name"
		exit 1
}

# Function build image takes 2 arguments: 1. username/repo_name 2. tag_name 3. Dockerfile
# Example: build_image johndoe/repo_name pytorch2.2.2 ubuntu22.04.Dockerfile
build_image() {
    echo "Building image $2"
    TAG_NAME=$2
    ALL_BUILD_ARGS="--build-arg CUDA_VERSION=${CUDA_VERSION} --build-arg PYTHON_VERSION=${PYTHON_VERSION} --build-arg PYTORCH_VERSION=${PYTORCH_VERSION}"
    echo "docker buildx build ${ALL_BUILD_ARGS} -t ${1}:${TAG_NAME} -f $3 ."
    docker buildx build --push ${ALL_BUILD_ARGS} -t ${1}:${TAG_NAME} -f $3 .
}

CUDA_VERSION="12.4.1"
PYTHON_VERSION="3.11"
PYTORCH_VERSION="2.4.1"
# echo "Building image pytorch2.3.1-cuda12.2"
TAG_NAME=jammy-torch${PYTORCH_VERSION}-cuda${CUDA_VERSION}
build_image $1 $TAG_NAME ubuntu22.04/cuda.Dockerfile

PYTHON_VERSION="3.10"
TAG_NAME=jammy-torch${PYTORCH_VERSION}-humble
build_image $1 $TAG_NAME ubuntu22.04/ros2.Dockerfile

TAG_NAME=jammy-torch${PYTORCH_VERSION}-cuda${CUDA_VERSION}-humble
build_image $1 $TAG_NAME ubuntu22.04/cuda-ros2.Dockerfile

PYTHON_VERSION="3.11"
TAG_NAME=jammy-torch${PYTORCH_VERSION}
build_image $1 $TAG_NAME ubuntu22.04/Dockerfile

TAG_NAME=latest
build_image $1 $TAG_NAME ubuntu22.04/Dockerfile
