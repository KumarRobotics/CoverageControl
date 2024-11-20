\page lpac LPAC Neural Network
\tableofcontents

# Preliminaries
We will organize files in a **workspace** directory: `${CoverageControl_ws}` (e.g., ~/CoverageControl\_ws).

Download and extract the file `lpac_CoverageControl.tar.gz` to the workspace directory.
The file can be downloaded from the repository [releases](https://github.com/KumarRobotics/CoverageControl/releases).
```bash
tar -xvzf lpac_CoverageControl.tar.gz -C ${CoverageControl_ws}
```
This will create a directory `lpac` in the workspace directory.
The directory structure is as follows:
```bash
${CoverageControl_ws}/
└── lpac/
    ├── data/   # To store datasets
    ├── envs/   # Environment files
    ├── eval/   # Results of evaluation
    ├── models/ # Trained models
    └── params/ # Parameters for training and evaluation
```

The models folder already contains a trained LPAC model for a 1024x1024 environment with 32 robots, 32 features, and 128 communication radius.

# Dataset Generation

There are two ways to classes for dataset generation located in `python/data_generation/`
1. `simple_data_generation.py`
2. `data_generation.py`

They are similar, except that `data_generation.py` splits the dataset into training, validation, and test sets.

To generate a dataset, run the following command:
```bash
python python/data_generation/data_generation.py \
       ${CoverageControl_ws}/lpac/params/data_params.toml
```

A sample `data_params.toml` file is also provided in the `params` directory of the repository.
See the file for details on the parameters.
The class will use a `coverage_control_params.toml` configuration file to generate environments and then use the `ClairvoyantCVT` algorithm to generate the dataset.

The `simple_data_generation.py` is useful for generating a large dataset in parts and then combining them into a single dataset.
See `python/utils/process_data.sh` and `python/utils/dataset_utils.py` for tools to process and combine datasets.

# Training

To train the LPAC model, run the following command:
```bash
python python/training/train_lpac.py \
       ${CoverageControl_ws}/lpac/params/learning_params.toml 1024
```

The second argument is the environment size, used to normalize the input features.
A sample `learning_params.toml` file is also provided in the `params` directory of the repository. See the file for details on the parameters.

# Evaluation
There are two scripts for evaluation located in `python/evaluators/`
1. [eval_single_env.py](python/evaluators/eval_single_env.py)
2. [eval.py](python/evaluators/eval.py)

`eval_single_env.py` evaluates a single environment and `eval.py` evaluates multiple environments.

To evaluate a trained model, run the following command:
```bash
python python/evaluators/eval.py \
       ${CoverageControl_ws}/lpac/params/eval.toml
```
or
```bash
python python/evaluators/eval_single_env.py \
       ${CoverageControl_ws}/lpac/params/eval_single.toml
```

The `eval.toml` and `eval_single.toml` files are also provided in the `params` directory of the repository.
