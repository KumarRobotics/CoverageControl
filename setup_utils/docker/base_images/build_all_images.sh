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
    docker buildx build --no-cache -t ${1}:${TAG_NAME} -f $3 .
    docker push ${1}:${TAG_NAME}
}

echo "Building image pytorch2.2.2-cuda12.2"
TAG_NAME=pytorch2.2.2-cuda12.2.2
# build_image $1 $TAG_NAME ubuntu22.04-cuda.Dockerfile

echo "Building image pytorch2.2.2-ros2humble"
TAG_NAME=pytorch2.2.2-ros2humble
build_image $1 $TAG_NAME ubuntu22.04-ros2.Dockerfile

echo "Building image pytorch2.2.2-cuda12.2.2-ros2humble"
TAG_NAME=pytorch2.2.2-cuda12.2.2-ros2humble
build_image $1 $TAG_NAME ubuntu22.04-cuda-ros2.Dockerfile

echo "Building image pytorch2.2.2"
TAG_NAME=pytorch2.2.2
build_image $1 $TAG_NAME ubuntu22.04.Dockerfile

