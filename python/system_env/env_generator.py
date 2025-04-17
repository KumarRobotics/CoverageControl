"""
Generates large number of random environments
"""

# @ file env_generator.py
# @brief Generates large number of random environments

import os
import pathlib
import argparse

from coverage_control import IOUtils
from coverage_control import Parameters
from coverage_control import WorldIDF
from coverage_control import CoverageSystem


def main(args):
    """
    Generates large number of random environments
    """

    env_config_file = IOUtils.sanitize_path(args.config)
    cc_params = Parameters(env_config_file)

    envs_path = pathlib.Path(IOUtils.sanitize_path(args.env_dir))

    if not os.path.exists(envs_path):
        print(f"Creating directory {envs_path}")
        os.makedirs(envs_path)

    pad_width = len(str(args.num_envs - 1))
    for i in range(args.num_envs):
        pos_file = str(envs_path / f"{i:0{pad_width}}.pos")
        env_file = str(envs_path / f"{i:0{pad_width}}.env")
        map_file = str(envs_path / f"{i:0{pad_width}}.map")
        if os.path.isfile(env_file) and os.path.isfile(pos_file) and args.use_existing == True:
            world_idf = WorldIDF(cc_params, env_file)
            cc_system = CoverageSystem(cc_params, world_idf, pos_file)
        else:
            cc_system = CoverageSystem(cc_params)

        cc_system.WriteEnvironment(pos_file, env_file)
        cc_system.WriteWorldMap(map_file)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generates large number of random environments"
    )
    parser.add_argument("config", type=str, help="Environment configuration file")
    parser.add_argument("num_envs", type=int, help="Number of environments to generate")
    parser.add_argument("env_dir", type=str, help="Directory to save the environments")
    parser.add_argument("--use_existing", type=bool, default=False, help="Use existing env if files exist")
    parser.add_argument("--write_pos", type=bool, default=True, help="Write initial positions of robots")
    parser.add_argument("--write_env", type=bool, default=True, help="Write idf info of environment")
    parser.add_argument("--write_map", type=bool, default=True, help="Write environment map")

    args = parser.parse_args()

    main(args)
