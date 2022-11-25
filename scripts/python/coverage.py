import numpy as np
import math
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points
from pyCoverageControl import PointVector # for defining list of points

# We can visualize the map in python
# Doesn't look great as there aren't many distributions
# The axes are interchanged and y is flipped. How to fix this?
# Make tick positions multiples of 100
import matplotlib.pyplot as plt
import seaborn as sns
colormap = sns.color_palette("light:b", as_cmap=True)
def plot_map(map):
    ax = sns.heatmap(map, cmap=colormap)
    ax.set_box_aspect(1) # Throws an error on lower versions of seaborn?
    plt.show()


####################
## CoverageSystem ##
####################
from pyCoverageControl import CoverageSystem
# Create an environment with random distributions and robots at random start positions
# See the parameter file for the range of random distributions
num_gaussians = 100
num_robots = 2
env = CoverageSystem(num_gaussians, num_robots)
map = env.GetWorldIDF()
plot_map(map)

# We can provide controls or update the positions directly
# The size of these vectors should be the same as the number of robots
new_robot_positions = PointVector()
new_robot_positions.append(Point2(100,100))
new_robot_positions.append(Point2(150,150))
env.UpdateRobotPositions(new_robot_positions)

control_directions = PointVector()
control_directions.append(Point2(math.sin(math.pi/4), math.cos(math.pi/4)))
control_directions.append(Point2(math.sin(math.pi/6), math.cos(math.pi/6)))
speeds = pyCoverageControl.DblVector()
speeds.append(1)
speeds.append(1)
env.StepControl(control_directions, speeds)

# Get current global robot positions
robot_positions = env.GetRobotPositions()
print(type(robot_positions))
print(robot_positions[0])

# Get local map of a robot
robot_id = 0
local_map = env.GetRobotLocalMap(robot_id) 
plot_map(local_map)

# Get sensor view of a robot
robot_id = 0
sensor_view = env.GetRobotSensorView(robot_id) 
plot_map(sensor_view)


# Examples of each class

############
## Point2 ##
############
# Point2 has two atrributes x and y
pt = Point2(0,0) # Needs two arguments for x and y
print(pt)
pt.x = 5
pt.y = 10
print(pt.x)

point_list = PointVector()
point_list.append(pt)
pt1 = Point2(512, 512)
point_list.append(pt1)

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

###############################
## Additional CoverageSystem ##
###############################

# We can also give the start positions of the robots
robot_positions = PointVector()
robot_positions.append(Point2(100,100))
robot_positions.append(Point2(150,150))
env1 = CoverageSystem(world_idf, robot_positions)

# We can also specify distributions and robots
bnd_list = pyCoverageControl.BNDVector()
bnd_list.append(dist1)
bnd_list.append(dist2)
bnd_list.append(dist3)
env2 = CoverageSystem(bnd_list, robot_positions)

