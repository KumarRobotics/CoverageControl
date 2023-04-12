import sys
import os
import math
import time
import csv
import numpy as np
import pyCoverageControl # Main library
from pyCoverageControl import Point2, PointVector # for defining points
from pyCoverageControl import BivariateNormalDistribution as BND
from pyCoverageControl import WorldIDF, CoverageSystem

features_filename_csv = "data/features.csv"

params = pyCoverageControl.Parameters()

world_idf = WorldIDF(params)

# sigma and peak can be randomize, e.g., randomly choose a sigma between params.pMinSigma and params.pMaxSigma
sigma = params.pMinSigma
peak = params.pMinPeak

with open(features_filename_csv, newline='') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=',')
    for row in csvreader:
        world_idf.AddNormalDistribution(BND(Point2(float(row[0]), float(row[1])), sigma, peak))

# The positions of the robots can be randomized
robots_positions = PointVector()
robots_positions.append(Point2(100.0, 100.0))
robots_positions.append(Point2(50.0, 150.0))

env = CoverageSystem(params, world_idf, robots_positions)
env.PlotInitMap('./', 'init_map')

