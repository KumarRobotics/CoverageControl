#!/bin/bash

# Set the script directory based on the CoverageControl_ws environment variable
SCRIPT_DIR="${CoverageControl_ws}/src/CoverageControl/python"

# Set the parameters directory based on the environment variable
PARAMS_DIR="${CoverageControl_ws}/lpac/params/"

# Define the parameter file names
DATA_PARAMS_FILE="data_params.toml"
# DATA_GEN_ALGORITHM="--algorithm CentralizedCVT"
LEARNING_PARAMS_FILE="learning_params.toml"
EVAL_PARAMS_FILE="eval.toml"

# Function to print messages in red
print_error() {
    echo -e "\033[0;31m$1\033[0m"
}

# Function to print the completion time and duration of each command
print_completion() {
    local command_name=$1
    local start_time=$2
    local end_time=$(date +%s)
    echo "Completed $command_name at $(date +'%Y-%m-%d %H:%M:%S')"
    echo "Duration of $command_name: $((end_time - start_time)) seconds"
}

# Run a command with timing and error checking
run_command() {
    local command=$1
    local name=$2
    start_time=$(date +%s)
    echo "Starting $name at $(date +'%Y-%m-%d %H:%M:%S')"

    # Execute the command
    eval $command
    if [ $? -ne 0 ]; then
        print_error "$name failed."
        exit 1
    fi

    print_completion "$name" $start_time
}

# Ensure the environment variable CoverageControl_ws is set
if [ -z "${CoverageControl_ws}" ]; then
    print_error "Environment variable CoverageControl_ws is not set."
    exit 1
fi

# Data generation in parts
# for i in {0..4}
# do
#   echo "Running data generation for $i"
#   run_command "python ${SCRIPT_DIR}/data_generation/data_generation.py ${PARAMS_DIR}/${DATA_PARAMS_FILE} --append-dir data/$i --split False" "Data Generation"
# done
# Edit and execute process_data.sh

# Running the data generation script
run_command "python ${SCRIPT_DIR}/data_generation/data_generation.py ${PARAMS_DIR}/${DATA_PARAMS_FILE} ${DATA_GEN_ALGORITHM} --split True" "Data Generation"

# Running the training script
run_command "python ${SCRIPT_DIR}/training/train_lpac.py ${PARAMS_DIR}/${LEARNING_PARAMS_FILE} 1024" "Model Training"

# Running the evaluation script
run_command "python ${SCRIPT_DIR}/evaluators/eval.py ${PARAMS_DIR}/${EVAL_PARAMS_FILE}" "Model Evaluation"
