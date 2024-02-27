print_usage() {
		echo "Usage: ./build_all_images.sh <dockerhub_username/repo_name>"
		echo "Example: ./build_all_images.sh johndoe/repo_name"
		exit 1
}
if [ "$#" -ne 1 ]; then
		print_usage
fi
echo "Building image pytorch2.2.1"
docker buildx build -t ${1}:pytorch2.2.1 -f ubuntu22.04.Dockerfile .
docker image tag ${1}:pytorch2.2.1 ghcr.io/${1}:pytorch2.2.1
docker push ${1}:pytorch2.2.1
docker push ghcr.io/${1}:pytorch2.2.1
echo "Building image pytorch2.2.1-cuda12.3.1"
docker buildx build -t ${1}:pytorch2.2.1-cuda12.3.1 -f ubuntu22.04-cuda.Dockerfile .
docker image tag ${1}:pytorch2.2.1-cuda12.3.1 ghcr.io/${1}:pytorch2.2.1-cuda12.3.1
docker push ${1}:pytorch2.2.1-cuda12.3.1
docker push ghcr.io/${1}:pytorch2.2.1-cuda12.3.1
echo "Building image pytorch2.2.1-ros2humble"
docker buildx build -t ${1}:pytorch2.2.1-ros2humble -f ubuntu22.04-ros2.Dockerfile .
docker image tag ${1}:pytorch2.2.1-ros2humble ghcr.io/${1}:pytorch2.2.1-ros2humble
docker push ${1}:pytorch2.2.1-ros2humble
docker push ghcr.io/${1}:pytorch2.2.1-ros2humble
echo "Building image pytorch2.2.1-cuda12.3.1-ros2humble"
docker buildx build -t ${1}:pytorch2.2.1-cuda12.3.1-ros2humble -f ubuntu22.04-cuda-ros2.Dockerfile .
docker image tag ${1}:pytorch2.2.1-cuda12.3.1-ros2humble ghcr.io/${1}:pytorch2.2.1-cuda12.3.1-ros2humble
docker push ${1}:pytorch2.2.1-cuda12.3.1-ros2humble
docker push ghcr.io/${1}:pytorch2.2.1-cuda12.3.1-ros2humble
