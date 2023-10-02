IMAGE_NAME=agarwalsaurav/gnn:latest
COVERAGECONTROL_WS=${1}

COVERAGECONTROL_VOLUME="--volume=${COVERAGECONTROL_WS}:/opt/CoverageControl_ws:rw"
CONTAINERNAME="gnn-${USER}"

docker run -it \
    --name=$CONTAINERNAME \
    --gpus all \
    --net=host \
    --privileged \
    ${COVERAGECONTROL_VOLUME} \
    --ipc=host \
    ${IMAGE_NAME}\
    bash
