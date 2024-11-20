print_usage() {
		echo "Usage: ./build_all_images.sh <username/repo_name>"
		echo "Example: ./build_all_images.sh johndoe/repo_name"
		echo "Example: ./build_all_images.sh ghcr.io/johndoe/repo_name"
		exit 1
}

if [ "$#" -ne 1 ]; then
    print_usage
fi

# Function build image takes 2 arguments: 1. username/repo_name 2. tag_name 3. Dockerfile
# Example: build_image johndoe/repo_name pytorch2.2.2 ubuntu22.04.Dockerfile
build_image() {
    echo "Building image $2"
    TAG_NAME=$2
    ALL_BUILD_ARGS="--no-cache --build-arg CUDA_VERSION=${CUDA_VERSION} --build-arg PYTHON_VERSION=${PYTHON_VERSION} --build-arg PYTORCH_VERSION=${PYTORCH_VERSION}"
    echo "docker buildx build ${ALL_BUILD_ARGS} -t ${1}:${TAG_NAME} -f $3 ."
    docker buildx build --push ${ALL_BUILD_ARGS} -t ${1}:${TAG_NAME} -f $3 .
    if [ $? -ne 0 ]; then
        echo "Failed to build image $2"
        exit 1
    fi
}

CUDA_VERSION="12.4.1"
PYTHON_VERSION="3.11"
PYTORCH_VERSION="2.5.1"
TAG_NAME=jammy-torch${PYTORCH_VERSION}-cuda${CUDA_VERSION}
build_image $1 $TAG_NAME ubuntu22.04/cuda.Dockerfile

PYTHON_VERSION="3.10"
TAG_NAME=jammy-torch${PYTORCH_VERSION}-humble
build_image $1 $TAG_NAME ubuntu22.04/ros2.Dockerfile

PYTHON_VERSION="3.10"
TAG_NAME=jammy-torch${PYTORCH_VERSION}-cuda${CUDA_VERSION}-humble
build_image $1 $TAG_NAME ubuntu22.04/cuda-ros2.Dockerfile

PYTHON_VERSION="3.11"
TAG_NAME=jammy-torch${PYTORCH_VERSION}
build_image $1 $TAG_NAME ubuntu22.04/Dockerfile

CUDA_VERSION="12.6.2"
PYTHON_VERSION="3.12"
PYTORCH_VERSION="2.5.1"
TAG_NAME=noble-torch${PYTORCH_VERSION}-cuda${CUDA_VERSION}
build_image $1 $TAG_NAME ubuntu24.04/cuda.Dockerfile

TAG_NAME=noble-torch${PYTORCH_VERSION}-jazzy
build_image $1 $TAG_NAME ubuntu24.04/ros2.Dockerfile

TAG_NAME=noble-torch${PYTORCH_VERSION}-cuda${CUDA_VERSION}-jazzy
build_image $1 $TAG_NAME ubuntu24.04/cuda-ros2.Dockerfile

TAG_NAME=noble-torch${PYTORCH_VERSION}
build_image $1 $TAG_NAME ubuntu24.04/Dockerfile

TAG_NAME=latest
build_image $1 $TAG_NAME ubuntu24.04/Dockerfile

docker buildx build --platform linux/arm64 -t ${1}:arm64-jammy-torch2.5.1-humble -f ubuntu22.04/arm64-ros2.Dockerfile --push .
docker buildx build --platform linux/arm64 -t ${1}:arm64-noble-torch2.5.1-jazzy -f ubuntu24.04/arm64-ros2.Dockerfile --push .
