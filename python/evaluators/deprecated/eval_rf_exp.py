import os
import numpy as np
import sys

import torch

from CoverageControlTorch.data_loaders import data_loader_utils as dl_utils
from eval import Evaluator

if __name__ == "__main__":

    rf_dir = "/root/CoverageControl_ws/results/1024/rf_exp/"

    r = int(sys.argv[1])
    r_dir = rf_dir + "r" + str(r);
    # Check if directory exists
    if not os.path.isdir(r_dir):
        print("Directory does not exist: " + r_dir)
        exit(1)
    for f in range(8, 65, 8):
        f_dir = r_dir + "/" + str(f)
        # Check if directory exists
        if not os.path.isdir(f_dir):
            print("Directory does not exist: " + f_dir)
            exit(1)

        env_file = f_dir + "/env_params.toml"
        eval_file = f_dir + "/eval.toml"

        env_config = dl_utils.LoadToml(env_file)
        if env_config["pNumRobots"] != r:
            print("Incorrect number of robots: " + f_dir)
            exit(1)
        if env_config["pNumFeatures"] != f:
            print("Incorrect number of features: " + f_dir)
            exit(1)

        eval_dir = f_dir + "/"
        eval_config = dl_utils.LoadToml(eval_file)
        if eval_config["EvalDir"] != eval_dir:
            print("Incorrect eval directory: " + f_dir)
            exit(1)
        
        if eval_config["EnvironmentConfig"] != env_file:
            print("Incorrect environment config: " + f_dir)
            exit(1)

        evaluator = Evaluator(eval_config)
        evaluator.Evaluate()


    # config_file = sys.argv[1]
    # config = dl_utils.LoadToml(config_file)
    # orig_name = config["Controllers"][0]["Name"]
    # orig_dir = os.path.dirname(config["Controllers"][0]["ModelFile"])

    # # config["Controllers"][0]["Name"] = orig_name + "/pre"
    # # config["Controllers"][0]["ModelFile"] = orig_dir + "/model_1024.pt"
    # # print(config)
    # # evaluator = Evaluator(config)
    # # evaluator.Evaluate()

    # config["Controllers"][0]["Name"] = orig_name + "/curr"
    # config["Controllers"][0]["ModelFile"] = orig_dir + "/model_curr.pt"
    # print(config)
    # evaluator = Evaluator(config)
    # evaluator.Evaluate()

    # config["Controllers"][0]["Name"] = orig_name + "/model"
    # config["Controllers"][0]["ModelFile"] = orig_dir + "/model.pt"
    # print(config)
    # evaluator = Evaluator(config)
    # evaluator.Evaluate()

    # for i in range(0, 100, 5):
    #     config["Controllers"][0]["Name"] = orig_name + "/" + str(i)
    #     config["Controllers"][0]["ModelFile"] = orig_dir + "/model_epoch" + str(i) + ".pt"
    #     print(config)
    #     evaluator = Evaluator(config)
    #     evaluator.Evaluate()

