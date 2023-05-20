import sys
import os
import math
import time
import numpy as np
import pyCoverageControl # Main library
from pyCoverageControl import Point2, PointVector # for defining points
from pyCoverageControl import BivariateNormalDistribution as BND
from pyCoverageControl import WorldIDF, CoverageSystem
# Algorithms available:
# LloydGlobalOnline
# LloydLocalVornoi
# OracleGlobalOffline
# OracleSimulExploreExploit
from pyCoverageControl import OracleSimulExploreExploit as CoverageAlgorithm

features_filename = "data/features"
robots_positions_filename = "data/robots_positions"

params = pyCoverageControl.Parameters()

# IDF is importance density field and it comprises features of interests
world_idf = WorldIDF(params, features_filename)

# CoverageSystem handles the environment and robots
env = CoverageSystem(params, world_idf, robots_positions_filename)

# Runs the coverage control algorithm
controller = CoverageAlgorithm(params, env.GetNumRobots(), env)

for i in range(0, params.pEpisodeSteps):
    # Step the controller to generate next set of actions
    cont_flag = controller.Step();
    # Get actions from the controller
    actions = controller.GetActions()
    # Send actions to the environment
    error_flag = env.StepActions(actions)

    if cont_flag == False:
        break
    if error_flag == True:
        print("Unexpected error")
        break

# print some metrics
print("Exploration ratio: " + str(env.GetExplorationRatio()) + " Weighted exploration ratio: " + str(env.GetWeightedExplorationRatio()))
print("Coverage objective: " + str('{:.2e}'.format(env.GetObjectiveValue())))


