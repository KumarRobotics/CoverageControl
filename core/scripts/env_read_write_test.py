import sys
import os
import math
import time
import numpy as np
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points
from pyCoverageControl import PointVector # for defining list of points
from pyCoverageControl import CoverageSystem

num_gaussians = 5
num_robots = 15

params_filename = "params/parameters.yaml"
dir = "./"

params = pyCoverageControl.Parameters(params_filename)
env = CoverageSystem(params, num_gaussians, num_robots)
map = env.GetWorldIDF()
env.WriteEnvironment(dir + "/env.pos", dir + "/env.idf")
env.PlotWorldMap(dir, "env")

world_idf = pyCoverageControl.WorldIDF(params, dir + "/env.idf")

env2 = CoverageSystem(params, world_idf, dir + "/env.pos")
env2.PlotWorldMap(dir, "env2")
map2 = env2.GetWorldIDF()

# Check if the two maps are the same
# maps are numpy two dimensional arrays
print("Maps are the same: ", np.array_equal(map, map2))


