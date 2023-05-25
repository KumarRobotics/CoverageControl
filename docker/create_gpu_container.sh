IMAGE_NAME=agarwalsaurav/gnn:cuda11.8-torch2.0.1-ubuntu22.04
COVERAGECONTROL_WS=${1}

COVERAGECONTROL_VOLUME="--volume=${COVERAGECONTROL_WS}:/root/CoverageControl_ws:rw"
CONTAINERNAME="gnn-${USER}-jammy"

docker run -it \
    --name=$CONTAINERNAME \
    --gpus all \
    --net=host \
    --privileged \
    ${COVERAGECONTROL_VOLUME} \
    --ipc=host \
    ${IMAGE_NAME}\
    bash
