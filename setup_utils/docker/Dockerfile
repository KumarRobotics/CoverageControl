ARG BASE_IMAGE=ghcr.io/kumarrobotics/pytorch_base
ARG BASE_TAG=pytorch2.2.1
ARG SETUP_ARGS="--with-cuda"

FROM ${BASE_IMAGE}:${BASE_TAG}
WORKDIR /app
ADD . .
RUN ["bash", "-c", "/app/setup.sh ${SETUP_ARGS}"]
RUN ${VENV_PATH}/bin/pip install .

COPY setup_utils/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# RUN export SETUP_ARGS=${SETUP_ARGS} && source ${VENV_PATH}/bin/activate && bash -c "./setup.sh ${SETUP_ARGS}"
