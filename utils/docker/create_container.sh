ORIG_INPUT_PARAMS="$@"
params="$(getopt -o d:n: -l directory: -l name:,with-cuda,with-ros,noble --name "$(basename "$0")" -- "$@")"

if [ $? -ne 0 ]
then
	echo "Parameter error"
	print_usage
fi

print_usage() {
	printf "bash $0 [-d|--directory <workspace directory>] [--with-cuda] [--with-ros]\n"
}

eval set -- "$params"
unset params

IMAGE_BASE_NAME=agarwalsaurav/pytorch_base
IMAGE_TAG=latest

NOBLE=false
while true; do
	case ${1} in
		-d|--directory) WS_DIR=("${2}");shift 2;;
		-n|--name) CONTAINER_NAME=("${2}");shift 2;;
		--with-cuda) CUDA_IMAGE=true;shift;;
		--with-ros) ROS_IMAGE=true;shift;;
		--noble) NOBLE=true;shift;;
		--) shift;break;;
		*) print_usage
			exit 1 ;;
	esac
done

CONTAINER_CC_WS="/workspace"

if [ -z ${WS_DIR} ]; then
	VOLUME_OPTION=""
else
	VOLUME_OPTION="-v ${WS_DIR}:${CONTAINER_CC_WS}:rw"
fi

if [[ ${NOBLE} == true ]]; then
  IMAGE_TAG="noble"
else
  IMAGE_TAG="jammy"
fi

IMAGE_TAG=${IMAGE_TAG}-torch2.5.1

if [[ ${CUDA_IMAGE} == true ]]; then
	CONTAINER_OPTIONS+="--gpus all "
  if [[ ${NOBLE} == true ]]; then
    IMAGE_TAG="${IMAGE_TAG}-cuda12.6.2"
  else
    IMAGE_TAG="${IMAGE_TAG}-cuda12.4.1"
  fi
fi

if [[ ${ROS_IMAGE} == true ]]; then
  if [[ ${NOBLE} == true ]]; then
    IMAGE_TAG="${IMAGE_TAG}-jazzy"
  else
    IMAGE_TAG="${IMAGE_TAG}-humble"
  fi
fi

IMAGE_NAME="${IMAGE_BASE_NAME}:${IMAGE_TAG}"
docker pull ${IMAGE_NAME}

if [ -z ${CONTAINER_NAME} ]; then
	CONTAINER_NAME="coverage-control-${USER}"
fi

docker run -it \
	--name=${CONTAINER_NAME} \
	${CONTAINER_OPTIONS} \
  --env=CoverageControl_ws=${CONTAINER_CC_WS} \
	--net=host \
	--privileged \
	--ipc=host \
	${VOLUME_OPTION} \
	--workdir=${CONTAINER_CC_WS} \
	${IMAGE_NAME} \
	bash
