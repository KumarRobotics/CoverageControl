"""
Generates large number of random environments
"""

# @ file env_generator.py
# @brief Generates large number of random environments

import os
import pathlib
import argparse

from coverage_control import Parameters
from coverage_control import CoverageSystem
from coverage_control import IOUtils


def main(env_config_file: str, num_envs: int, env_dir: str):
    """
    Generates large number of random environments
    """

    env_config_file = IOUtils.sanitize_path(env_config_file)
    cc_params = Parameters(env_config_file)

    envs_path = pathlib.Path(IOUtils.sanitize_path(env_dir))

    if not os.path.exists(envs_path):
        print(f"Creating directory {envs_path}")
        os.makedirs(envs_path)

    for i in range(num_envs):
        pos_file = str(envs_path / f"{i:04}.pos")
        env_file = str(envs_path / f"{i:04}.env")
        cc_system = CoverageSystem(cc_params)
        cc_system.WriteEnvironment(pos_file, env_file)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generates large number of random environments"
    )
    parser.add_argument("config", type=str, help="Environment configuration file")
    parser.add_argument("num_envs", type=int, help="Number of environments to generate")
    parser.add_argument("env_dir", type=str, help="Directory to save the environments")
    args = parser.parse_args()

    main(args.config, args.num_envs, args.env_dir)
