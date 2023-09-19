IMAGE_NAME=agarwalsaurav/gnn_ros
COVERAGECONTROL_WS=${1}

COVERAGECONTROL_VOLUME="--volume=${COVERAGECONTROL_WS}:/root/CoverageControl_ws:rw"
CONTAINERNAME="gnn-${USER}-ros"

docker run -it \
    --name=$CONTAINERNAME \
    --gpus all \
    --net=host \
    --privileged \
    ${COVERAGECONTROL_VOLUME} \
    --ipc=host \
    ${IMAGE_NAME}\
    bash
