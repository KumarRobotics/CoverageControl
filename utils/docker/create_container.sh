#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status,
# treat unset variables as errors, and ensure pipelines fail correctly.
set -euo pipefail

# ----------------------------
# Color Definitions
# ----------------------------
RED='\033[0;31m'    # Red
GREEN='\033[0;32m'  # Green
YELLOW='\033[0;33m' # Yellow
NC='\033[0m'        # No Color

# ----------------------------
# Function Definitions
# ----------------------------

# Function to display usage information
print_usage() {
  cat <<EOF
Usage: bash $(basename "$0") [OPTIONS]

Options:
  -d, --directory <workspace directory>  Specify the workspace directory.
  -n, --name <container name>            Specify the container name.

      --with-cuda                        Use CUDA-enabled image.
      --with-ros                         Include ROS in the image.
      --noble                            Use 'noble' tag (Ubuntu 24.04).
      --arm64                            Use ARM64 architecture image.

  -h, --help                             Show this help message and exit.

Examples:
  bash $(basename "$0") -d \${CoverageControl_ws} --with-cuda --with-ros
  bash $(basename "$0") --noble --arm64 -n my_container

Note:
  - If no directory is specified, the container will not mount any volume.
  - By default, the script uses the 'jammy' (Ubuntu 22.04) tag.
EOF
}

# Function to check if a command exists
command_exists() {
  command -v "$1" >/dev/null 2>&1
}

# Function to handle errors with colored output
error_exit() {
  echo -e "${RED}Error: $1${NC}" >&2
  exit 1
}

# Function to display informational messages
info_message() {
  echo -e "${GREEN}$1${NC}"
}

# Function to display warnings
warning_message() {
  echo -e "${YELLOW}Warning: $1${NC}"
}

# Ensure required commands are available
for cmd in docker getopt; do
  if ! command_exists "$cmd"; then
    error_exit "'$cmd' command is not found. Please install it before running this script."
  fi
done

if [[ $# -eq 0 ]]; then
  print_usage
  exit 1
fi

# Save original input parameters
ORIG_INPUT_PARAMS="$@"

# Define short and long options
SHORT_OPTS="d:n:h"
LONG_OPTS="directory:,name:,with-cuda,with-ros,noble,arm64,help"

# Parse options using getopt
PARSED_PARAMS=$(getopt -o "$SHORT_OPTS" -l "$LONG_OPTS" --name "$(basename "$0")" -- "$@") || {
  error_exit "Failed to parse arguments."
}

# Evaluate the parsed options
eval set -- "$PARSED_PARAMS"

# Initialize variables with default values
WS_DIR=""
CONTAINER_NAME=""
CUDA_IMAGE=false
ROS_IMAGE=false
NOBLE=false
ARM=false

IMAGE_BASE_NAME="agarwalsaurav/pytorch_base"
IMAGE_TAG="latest"
CONTAINER_OPTIONS=""
CONTAINER_CC_WS="/workspace"

# Process parsed options
while true; do
  case "$1" in
    -d|--directory)
      WS_DIR="$2"
      shift 2
      ;;
    -n|--name)
      CONTAINER_NAME="$2"
      shift 2
      ;;
    --with-cuda)
      CUDA_IMAGE=true
      shift
      ;;
    --with-ros)
      ROS_IMAGE=true
      shift
      ;;
    --noble)
      NOBLE=true
      shift
      ;;
    --arm64)
      ARM=true
      shift
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    *)
      error_exit "Unknown option: $1"
      ;;
  esac
done

# ----------------------------
# Configuration
# ----------------------------

# Set the base image tag based on options
if [[ "$NOBLE" == true ]]; then
  IMAGE_TAG="noble"
else
  IMAGE_TAG="jammy"
fi

IMAGE_TAG="${IMAGE_TAG}-torch2.5.1"

# Handle CUDA options
if [[ "$CUDA_IMAGE" == true ]]; then
  CONTAINER_OPTIONS+="--gpus all "
  if [[ "$NOBLE" == true ]]; then
    IMAGE_TAG="${IMAGE_TAG}-cuda12.6.2"
  else
    IMAGE_TAG="${IMAGE_TAG}-cuda12.4.1"
  fi
fi

# Handle ROS options
if [[ "$ROS_IMAGE" == true ]]; then
  if [[ "$NOBLE" == true ]]; then
    IMAGE_TAG="${IMAGE_TAG}-jazzy"
  else
    IMAGE_TAG="${IMAGE_TAG}-humble"
  fi
fi

# Handle ARM64 architecture
if [[ "$ARM" == true ]]; then
  if [[ "$NOBLE" == true ]]; then
    IMAGE_TAG="arm64-noble-torch2.5.1-jazzy"
  else
    IMAGE_TAG="arm64-jammy-torch2.5.1-humble"
  fi
fi

IMAGE_NAME="${IMAGE_BASE_NAME}:${IMAGE_TAG}"

# Pull the Docker image
info_message "Pulling Docker image: ${IMAGE_NAME}"
if ! docker pull "${IMAGE_NAME}"; then
  error_exit "Failed to pull Docker image: ${IMAGE_NAME}"
fi

# Set container name if not provided
if [[ -z "$CONTAINER_NAME" ]]; then
  CONTAINER_NAME="coverage-control-${USER}"
fi

# Set volume option if workspace directory is provided
if [[ -n "$WS_DIR" ]]; then
  if [[ ! -d "$WS_DIR" ]]; then
    error_exit "Workspace directory '$WS_DIR' does not exist."
  fi
  VOLUME_OPTION="-v ${WS_DIR}:${CONTAINER_CC_WS}:rw"
else
  VOLUME_OPTION=""
fi

# ----------------------------
# Docker Run Command
# ----------------------------

docker run -it --init \
  --name="${CONTAINER_NAME}" \
  ${CONTAINER_OPTIONS} \
  --env=CoverageControl_ws="${CONTAINER_CC_WS}" \
  --net=host \
  --privileged \
  --ipc=host \
  ${VOLUME_OPTION} \
  --workdir="${CONTAINER_CC_WS}" \
  "${IMAGE_NAME}" \
  bash

