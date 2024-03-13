import os
import numpy as np
import sys

import torch

import coverage_control as cc
from coverage_control import IOUtils
from eval import Evaluator

if __name__ == "__main__":

    config_file = sys.argv[1]
    config = IOUtils.load_yaml(config_file)
    orig_name = config["Controllers"][0]["Name"]
    orig_dir = os.path.dirname(config["Controllers"][0]["ModelFile"])

    # config["Controllers"][0]["Name"] = orig_name + "/pre"
    # config["Controllers"][0]["ModelFile"] = orig_dir + "/model_1024.pt"
    # print(config)
    # evaluator = Evaluator(config)
    # evaluator.Evaluate()

    config["Controllers"][0]["Name"] = orig_name + "/curr"
    config["Controllers"][0]["ModelFile"] = orig_dir + "/model_curr.pt"
    print(config)
    evaluator = Evaluator(config)
    evaluator.Evaluate()

    config["Controllers"][0]["Name"] = orig_name + "/model"
    config["Controllers"][0]["ModelFile"] = orig_dir + "/model.pt"
    print(config)
    evaluator = Evaluator(config)
    evaluator.Evaluate()

    for i in range(0, 100, 5):
        config["Controllers"][0]["Name"] = orig_name + "/" + str(i)
        config["Controllers"][0]["ModelFile"] = orig_dir + "/model_epoch" + str(i) + ".pt"
        print(config)
        evaluator = Evaluator(config)
        evaluator.Evaluate()

