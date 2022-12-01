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
num_robots = 50
env = CoverageSystem(params_, num_gaussians, num_robots)
map = env.GetWorldIDF()
robot_positions = env.GetRobotPositions()
print(type(map))
print(type(robot_positions))

voronoi_cells = env.GetVoronoiCells()

robot_id = 0

for step in range(0, params_.pEpisodeSteps):
    print(step)
    if (env.StepLloydDistributed()):
        break
###################
## Visualization ##
###################

plt.ion()

fig = plt.figure("Environment")
ax = sns.heatmap(map.transpose(), vmax=params_.pNorm, cmap=colormap, square=True)
ax.invert_yaxis()
nrow, ncol = map.shape
# septicks = 5 ** (math.floor(math.log(nrow, 5)) - 1)
septicks = 10 ** (math.floor(math.log(nrow, 10)) - 1)
plt.xticks(np.arange(0, nrow, septicks), np.arange(0, nrow, septicks))
plt.yticks(np.arange(0, ncol, septicks), np.arange(0, ncol, septicks))

plot_pos_x = np.array([])
plot_pos_y = np.array([])
for pos in robot_positions:
    plot_pos_x = np.append(plot_pos_x, pos.x / params_.pResolution)
    plot_pos_y = np.append(plot_pos_y, pos.y / params_.pResolution)

plot_robots, = ax.plot(plot_pos_x, plot_pos_y, 'go')

prev_robot_pos = robot_positions
fig_local = plt.figure("Local Map of Robot" + str(robot_id))
local_map = env.GetRobotLocalMap(robot_id)
local_ax = sns.heatmap(data=np.flip(local_map.transpose(),0), vmax=params_.pNorm, cmap=colormap, square=True)
cbar_ax = fig_local.axes[-1]# retrieve previous cbar_ax (if exists)

batch = 5
for step in range(0, round(params_.pEpisodeSteps/batch)):
    print(step)
    for kk in range(0, batch):
        if (env.StepLloydDistributed()):
            break
    robot_positions = env.GetRobotPositions()
    local_map = env.GetRobotLocalMap(robot_id)
    sns.heatmap(ax=local_ax,data=np.flip(local_map.transpose(), 0), vmax=params_.pNorm, cmap=colormap, square=True, cbar_ax=cbar_ax, xticklabels=[],yticklabels=[])
    local_ax.set_title("Robot [" + str(robot_id) + "] position: " + "{:.{}f}".format(robot_positions[robot_id].x, 2) + ", " +  "{:.{}f}".format(robot_positions[robot_id].y, 2))
    for i in range(0, num_robots):
        plot_pos_x[i] =  prev_robot_pos[i].x / params_.pResolution
        plot_pos_y[i] =  prev_robot_pos[i].y / params_.pResolution

    prev_robot_pos = robot_positions
    plot_robots.set_xdata(plot_pos_x)
    plot_robots.set_ydata(plot_pos_y)

    fig.canvas.draw()
    fig.canvas.flush_events()
    fig_local.canvas.draw()
    fig_local.canvas.flush_events()

