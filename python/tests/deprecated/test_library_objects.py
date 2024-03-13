import numpy as np
import math
import coverage_control # Main library
from coverage_control import Point2 # for defining points
from coverage_control import PointVector # for defining list of points

# We can visualize the map in python
import matplotlib.pylab as plt
import seaborn as sns
colormap = sns.color_palette("light:b", as_cmap=True)
def plot_map(map):
    ax = sns.heatmap(map.transpose(), vmax=1, cmap=colormap, square=True)
    ax.invert_yaxis()
    nrow, ncol = map.shape
    if(nrow > 50 and nrow < 500):
        septicks = 5 ** (math.floor(math.log(nrow, 5)) - 1)
    else:
        septicks = 10 ** (math.floor(math.log(nrow, 10)) - 1)
    plt.xticks(np.arange(0, nrow, septicks), np.arange(0, nrow, septicks))
    plt.yticks(np.arange(0, ncol, septicks), np.arange(0, ncol, septicks))
    plt.show()


################
## Parameters ##
################
# The parameters can be changed from python without needing to recompile.
# The parameters are given in config/parameters.yaml
# After changing the parameters, use the following function call to use the yaml file.
# Make sure the path of the file is correct
params_ = coverage_control.Parameters()

####################
## CoverageSystem ##
####################
from coverage_control import CoverageSystem
# Create an environment with random distributions and robots at random start positions
# See the parameter file for the range of random distributions
num_gaussians = 100
num_robots = 2
env = CoverageSystem(params_, num_gaussians, num_robots)
map = env.GetWorldIDF()
plot_map(map)

# We can provide controls or update the positions directly
# The size of these vectors should be the same as the number of robots
control_directions = PointVector()
control_directions.append(np.array([math.sin(math.pi/4), math.cos(math.pi/4)]))
control_directions.append(np.array([math.sin(math.pi/6), math.cos(math.pi/6)]))
speeds = coverage_control.DblVector()
speeds.append(1)
speeds.append(1)
env.StepControl(0, control_directions[0], speeds[0])
env.StepControl(1, control_directions[1], speeds[1])

# Update the local position of the robots
new_robot_positions = PointVector()
new_robot_positions.append(np.array([10, 10]))
new_robot_positions.append(np.array([11, 11]))
env.SetRobotPositions(new_robot_positions)

# Get current global robot positions
robot_positions = env.GetRobotPositions()
print(type(robot_positions))
print(robot_positions[0])
print(robot_positions[1])

# Get local map of a robot
robot_id = 0
local_map = env.GetRobotLocalMap(robot_id)
plot_map(local_map)

# Get sensor view of a robot
robot_id = 0
sensor_view = env.GetRobotSensorView(robot_id)
plot_map(sensor_view)

# Get Communication Map
comm_map = env.GetCommunicationMap(robot_id)
plot_map(comm_map)

# Examples of each class

############
## Point2 ##
############
# Point2 has two atrributes x and y
# The class can be essentially used as numpy array
pt = Point2(0,0) # Needs two arguments for x and y
print(pt)

point_list = PointVector()
point_list.append(pt)
pt1 = Point2(512, 512)
point_list.append(pt1)

#################################
## BivariateNormalDistribution ##
#################################
from coverage_control import BivariateNormalDistribution as BND # for defining bivariate normal distributions
dist1 = BND() # zero-mean, sigma = 1, circular

mean = Point2(100, 400)
sigma = 5
peak_val = 3
dist2 = BND(mean, sigma, peak_val) # circular gaussian

mean3 = Point2(500, 10)
sigma_skewed = Point2(1, 2)
rho = 0.5
dist3 = BND(mean3, sigma_skewed, rho, peak_val) # general BND

##############
## WorldIDF ##
##############
from coverage_control import WorldIDF # for defining world idf
world_idf = WorldIDF(params_)
world_idf.AddNormalDistribution(dist1); # Add a distribution to the idf
world_idf.AddNormalDistribution(dist2); # Add a distribution to the idf
world_idf.AddNormalDistribution(dist3); # Add a distribution to the idf
world_idf.GenerateMapCuda() # Generate map, use GenerateMap() for cpu version
world_idf.PrintMapSize()
# world_idf.WriteWorldMap("map.dat"); # Writes the matrix to the file. Map should have been generated before writing
map = world_idf.GetWorldMap(); # Get the world map as numpy nd-array. You can only "view", i.e., flags.writeable = False, flags.owndata = False
normalization_factor = world_idf.GetNormalizationFactor(); # Get the normalization factor used in the map
print(map.dtype)
print(map.flags)
print(type(map))
print(map[0, 0])
plot_map(map)

################
## RobotModel ##
################

from coverage_control import RobotModel
# RobotModel requires an initial start position and a WorldIDF
# It does not modify the WorldIDF but only queries it
start_pos = Point2(0, 0)
robot = RobotModel(params_, start_pos, world_idf)

# Get the start global position of the robot
robot_pos = robot.GetGlobalStartPosition()

# There are two ways to update the position of the robot (1) control (2) position update
control_dir = Point2(math.sin(math.pi/4), math.cos(math.pi/4))
speed = 5 # m/s
robot.StepControl(control_dir, speed)
# Second way: directly update pos of robot
new_pos = Point2(512, 512)
robot.SetRobotPosition(new_pos)

# Get the current global position of the robot
robot_pos = robot.GetGlobalCurrentPosition()

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
env1 = CoverageSystem(params_, world_idf, robot_positions)

# We can also specify distributions and robots
bnd_list = coverage_control.BNDVector()
bnd_list.append(dist1)
bnd_list.append(dist2)
bnd_list.append(dist3)
env2 = CoverageSystem(params_, bnd_list, robot_positions)

print("System map")
sys_map = env2.GetSystemMap()
print(type(sys_map))
print(sys_map.dtype)
print(sys_map.flags)
print(np.min(sys_map))
print((sys_map.dtype.byteorder))
