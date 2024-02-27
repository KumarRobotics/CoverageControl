ARG BASE_IMAGE=agarwalsaurav/pytorch_base
ARG BASE_TAG=pytorch2.2.1
ARG SETUP_ARGS="-p"

FROM ${BASE_IMAGE}:${BASE_TAG}
WORKDIR /app
ADD . .
ENV SETUP_ARGS=${SETUP_ARGS}

COPY setup_utils/install_dependencies.sh /install_dependencies.sh
RUN chmod +x /install_dependencies.sh
RUN /install_dependencies.sh

COPY setup_utils/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# RUN export SETUP_ARGS=${SETUP_ARGS} && source ${VENV_PATH}/bin/activate && bash -c "./setup.sh ${SETUP_ARGS}"
