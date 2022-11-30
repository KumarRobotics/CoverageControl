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

params_ = pyCoverageControl.Parameters('parameters.yaml')

num_gaussians = 100
num_robots = 50
env = CoverageSystem(params_, num_gaussians, num_robots)
map = env.GetWorldIDF()
robot_positions = env.GetRobotPositions()
print(type(map))
print(type(robot_positions))

voronoi_cells = env.GetVoronoiCells()

robot_id = 0

env.StepOracleN(1000)
# for step in range(0, params_.pEpisodeSteps):
#     print(step)
#     if (not env.StepOracle()):
#         break
###################
## Visualization ##
###################

plt.ion()

fig = plt.figure("Environment")
ax = sns.heatmap(map.transpose(), vmax=params_.pNorm, cmap=colormap, square=True)
ax.invert_yaxis()
nrow, ncol = map.shape
septicks = 5 ** (math.floor(math.log(nrow, 5)) - 1)
# septicks = 10 ** (math.floor(math.log(nrow, 10)) - 1)
plt.xticks(np.arange(0, nrow, septicks), np.arange(0, nrow, septicks))
plt.yticks(np.arange(0, ncol, septicks), np.arange(0, ncol, septicks))

plot_pos_x = np.array([])
plot_pos_y = np.array([])
for pos in robot_positions:
    plot_pos_x = np.append(plot_pos_x, pos.x / params_.pResolution)
    plot_pos_y = np.append(plot_pos_y, pos.y / params_.pResolution)

plot_robots, = ax.plot(plot_pos_x, plot_pos_y, 'go')

prev_robot_pos = robot_positions
fig_local = plt.figure("Oracle Map")
oracle_map = env.GetOracleMap()
local_ax = sns.heatmap(oracle_map.transpose(), vmax=params_.pNorm, cmap=colormap, square=True)
plot_oracle_robots, = local_ax.plot(plot_pos_x, plot_pos_y, 'go')
cbar_ax = fig_local.axes[-1]# retrieve previous cbar_ax (if exists)

batch = 20
cont_flag = True
for step in range(0, round(params_.pEpisodeSteps/batch)):
    if cont_flag == False:
        break
    print(step)
    cont_flag = env.StepOracleN(batch)
    robot_positions = env.GetRobotPositions()
    oracle_map = env.GetOracleMap()
    sns.heatmap(ax=local_ax,data=oracle_map.transpose(), vmax=params_.pNorm, cmap=colormap, square=True, cbar_ax=cbar_ax, xticklabels=[],yticklabels=[])
    local_ax.set_title("Oracle Map")
    for i in range(0, num_robots):
        plot_pos_x[i] =  prev_robot_pos[i].x / params_.pResolution
        plot_pos_y[i] =  prev_robot_pos[i].y / params_.pResolution

    prev_robot_pos = robot_positions
    plot_oracle_robots.set_xdata(plot_pos_x)
    plot_oracle_robots.set_ydata(plot_pos_y)

    fig.canvas.draw()
    fig.canvas.flush_events()
    fig_local.canvas.draw()
    fig_local.canvas.flush_events()

