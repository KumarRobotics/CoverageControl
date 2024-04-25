"""
A simple example of using WorldIDF
"""

import coverage_control

params = coverage_control.Parameters()
params.pNumRobots = 1
params.pNumFeatures = 5
params.pNumPolygons = 20
params.pWorldMapSize = 1024
params.pMaxVertices = 5
params.pPolygonRadius = 128

env = coverage_control.CoverageSystem(params)

env.PlotInitMap("init_map")
