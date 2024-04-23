#  This file is part of the CoverageControl library
#
#  Author: Saurav Agarwal
#  Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
#  Repository: https://github.com/KumarRobotics/CoverageControl
#
#  Copyright (c) 2024, Saurav Agarwal
#
#  The CoverageControl library is free software: you can redistribute it and/or
#  modify it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or (at your
#  option) any later version.
#
#  The CoverageControl library is distributed in the hope that it will be
#  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
#  Public License for more details.
#
#  You should have received a copy of the GNU General Public License along with
#  CoverageControl library. If not, see <https://www.gnu.org/licenses/>.
"""
A simple example of using WorldIDF
"""

import coverage_control
import numpy as np

params = coverage_control.Parameters()
params.pNumRobots = 1
params.pNumFeatures = 5
params.pNumPolygons = 20
params.pWorldMapSize = 1024
params.pMaxVertices = 5
params.pPolygonRadius = 128

env = coverage_control.CoverageSystem(params)

env.PlotInitMap("init_map")
