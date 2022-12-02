import sys
import math
import time
import numpy as np
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points
from pyCoverageControl import PointVector # for defining list of points
from pyCoverageControl import CoverageSystem

# We can visualize the map in python
import matplotlib.pylab as plt
import seaborn as sns
colormap = sns.color_palette("light:b", as_cmap=True)

params_ = pyCoverageControl.Parameters('../../params/parameters.yaml')

num_gaussians = 100
num_robots = 20
env = CoverageSystem(params_, num_gaussians, num_robots)
map = env.GetWorldIDF()

plt.ion()
robot_id = 0
fig_local = plt.figure("Local Map of Robot" + str(robot_id))
local_map = env.GetRobotLocalMap(robot_id)
local_ax = sns.heatmap(data=np.flip(local_map.transpose(),0), vmax=params_.pNorm, cmap=colormap, square=True)
cbar_ax = fig_local.axes[-1]# retrieve previous cbar_ax (if exists)

# We use the function StepDataGenerationLocal to drive the robots around in the world
# The CoverageSystem maintains an oracle_map_ which has the information of the world_idf_ at the locations visited by any of the robots. For locations that haven't been visited yet, the importance is set to a Parameters::pUnknownImportance
# The StepDataGenerationLocal uses an Oracle which decides where the robots should move based on the oracle_map_ described above. It performs a Voronoi Tessalation will ALL the robots, then computes the centroid, solves the assignment problem, and then steps the robots towards their respective assigned centroids.
# The difference from standard methods is that this Oracle does not assume that the entire map is known. It uses only the information that is make available by the robots.
# The StepDataGenerationLocal function has an input num_steps. This allows us to step several times before exchanging data in order to get sufficiently varying data.
# Thus we can simulate several times on the same environment to get different dataset.
# For the training of the CNN, each robot is a unique data
# For each environment, lets say we can simulate upto pEpisodeSteps=1000, as after that the robots might either converge or just osscilate and no new data is generated.
# So for an environment, we take num_steps=10 for pEpisodeSteps/num_steps times. So we will end up with a dataset of num_robots * pEpisodeSteps/num_steps unique data

num_steps = 10
print(round(params_.pEpisodeSteps/num_steps))
for iter in range(0, round(params_.pEpisodeSteps/num_steps)):
    # returns true if the robots have converged
    cont_flag = env.StepDataGenerationLocal(num_steps)
    if(not cont_flag):
        break
    positions = env.GetRobotPositions()
    # print(positions[robot_id])
    for iRobot in range(0, num_robots):
        robot_local_map = env.GetRobotLocalMap(iRobot)
        communication_map = env.GetCommunicationMap(iRobot)
        voronoi_cell = env.GetVoronoiCell(iRobot)
        centroid = voronoi_cell.centroid
        mass = voronoi_cell.mass
        # Do something with the data
    robot_local_map = env.GetRobotLocalMap(robot_id)
    sns.heatmap(ax=local_ax,data=np.flip(robot_local_map.transpose(), 0), vmax=params_.pNorm, cmap=colormap, square=True, cbar_ax=cbar_ax, xticklabels=[],yticklabels=[])

    fig_local.canvas.draw()
    fig_local.canvas.flush_events()
