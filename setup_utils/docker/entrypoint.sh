#!/bin/env bash
set -e

source ${VENV_PATH}/bin/activate
bash /app/setup.sh ${SETUP_ARGS}

/bin/bash
