import numpy as np
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points

# We can visualize the map in python
# Doesn't look great as there aren't many distributions
# The axes are interchanged and y is flipped. How to fix this?
# Make tick positions multiples of 100
import matplotlib.pyplot as plt
import seaborn as sns
colormap = sns.color_palette("light:b", as_cmap=True)
def plot_map(map):
    ax = sns.heatmap(map, cmap=colormap)
    ax.set_box_aspect(1)
    plt.show()

############
## Point2 ##
############
# Point2 has two atrributes x and y
pt = Point2(0,0) # Needs two arguments for x and y
print(pt)
pt.x = 5
pt.y = 10
print(pt.x)


#################################
## BivariateNormalDistribution ##
#################################
from pyCoverageControl import BivariateNormalDistribution as BND # for defining bivariate normal distributions
dist1 = BND() # zero-mean, sigma = 1, circular

mean = Point2(512, 512)
sigma = 5
peak_val = 3
dist2 = BND(mean, sigma, peak_val) # circular gaussian

sigma_skewed = Point2(1, 2)
rho = 2
dist3 = BND(mean, sigma_skewed, rho, peak_val) # general BND

##############
## WorldIDF ##
##############
from pyCoverageControl import WorldIDF # for defining world idf
world_idf = WorldIDF()
world_idf.AddNormalDistribution(dist2); # Add a distribution to the idf
world_idf.GenerateMapCuda() # Generate map, use GenerateMap() for cpu version
world_idf.PrintMapSize()
# world_idf.WriteWorldMap("map.dat"); # Writes the matrix to the file. Map should have been generated before writing
map = world_idf.GetWorldMap(); # Get the world map as numpy nd-array. You can only "view", i.e., flags.writeable = False, flags.owndata = False
max_val = world_idf.GetMaxValue(); # Get the maximum importance value for the entire map. Useful for normalization
print(map.dtype)
print(map.flags)
print(type(map))
print(map[0, 0])
plot_map(map)

################
## RobotModel ##
################

import math
from pyCoverageControl import RobotModel
# RobotModel requires an initial start position and a WorldIDF
# It does not modify the WorldIDF but only queries it
start_pos = Point2(0, 0)
robot = RobotModel(start_pos, world_idf)

# Get the start global position of the robot
robot_pos = robot.GetGlobalStartPosition() 

# There are two ways to update the position of the robot (1) control (2) position update
control_dir = Point2(math.sin(math.pi/4), math.cos(math.pi/4))
speed = 5 # m/s
robot.StepControl(control_dir, speed)
# Second way: directly update pos of robot
new_pos = Point2(512, 512)
robot.UpdateRobotPosition(new_pos)

# Get the current global position of the robot
robot_pos = robot.GetGlobalCurrentPosition() 

# Get all local positions of the robot
all_robot_positions = robot.GetAllPositions()
print(type(all_robot_positions))
print(all_robot_positions[0])

# Get the local map of the robot
robot_map = robot.GetRobotLocalMap()
plot_map(robot_map)

# Get the sensor view of the robot
sensor_view = robot.GetSensorView()
plot_map(sensor_view)
