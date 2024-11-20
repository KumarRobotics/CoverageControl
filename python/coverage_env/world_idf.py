"""
A simple example of using WorldIDF
"""

import sys
import coverage_control

if len(sys.argv) > 1:
    params_file = sys.argv[1]
    params = coverage_control.Parameters(params_file)
    params.pPlotScale = 2
    params.pTruncationBND = 2
else:
    params = coverage_control.Parameters()
    params.pNumRobots = 1
    params.pNumGaussianFeatures = 5
    params.pNumPolygons = 20
    params.pWorldMapSize = 1024
    params.pMaxVertices = 5
    params.pPolygonRadius = 128

env = coverage_control.CoverageSystem(params)

env.PlotInitMap("init_map")
