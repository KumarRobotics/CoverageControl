import sys
import math
import time
import numpy as np
from sklearn.metrics import pairwise_distances as pwdist
from scipy.optimize import linear_sum_assignment
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points
from pyCoverageControl import PointVector # for defining list of points
from pyCoverageControl import CoverageSystem
from pyCoverageControl import OracleGlobalOffline
from multiprocessing import Pool

# We can visualize the map in python
import matplotlib.pylab as plt
import seaborn as sns
colormap = sns.color_palette("light:b", as_cmap=True)


params_ = pyCoverageControl.Parameters('params/parameters.yaml')

num_gaussians = 100
num_robots = 20

count = 0
robot_id = 0

# Need to modify this function
def write_npz(iRobot):
    np.savez_compressed('../../../data/gnn_data/data_' + f'{(count * num_robots + iRobot):07d}' + '.npz', local_map = env.GetRobotLocalMap(iRobot), communication_map = env.GetCommunicationMap(iRobot), label=np.concatenate((env.GetVoronoiCell(iRobot).centroid, [env.GetVoronoiCell(iRobot).mass])))

num_steps = 0
env = CoverageSystem(params_, num_gaussians, num_robots)

oracle = OracleGlobalOffline(params_, num_robots, env)
oracle.ComputeGoals()
goals = oracle.GetGoals()
voronoi_cells = oracle.GetVoronoiCells()
robot_positions = env.GetRobotPositions()
map = env.GetWorldIDF()

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
    plot_pos_x = np.append(plot_pos_x, pos[0] / params_.pResolution)
    plot_pos_y = np.append(plot_pos_y, pos[1] / params_.pResolution)
plot_robots, = ax.plot(plot_pos_x, plot_pos_y, 'go')

centroid_x = np.array([])
centroid_y = np.array([])

plot_cells = []
for vcell in voronoi_cells:
    cell = vcell.cell
    plot_pt_x = np.array([])
    plot_pt_y = np.array([])
    for pt in cell:
        plot_pt_x = np.append(plot_pt_x, pt[0] / params_.pResolution)
        plot_pt_y = np.append(plot_pt_y, pt[1] / params_.pResolution)
    plot_pt_x = np.append(plot_pt_x, plot_pt_x[0])
    plot_pt_y = np.append(plot_pt_y, plot_pt_y[0])
    plot_cell, = ax.plot(plot_pt_x, plot_pt_y, 'r')
    plot_cells.append(plot_cell)
    centroid_x = np.append(centroid_x, vcell.centroid[0])
    centroid_y = np.append(centroid_y, vcell.centroid[1])
plot_centroids, = ax.plot(centroid_x, centroid_y, 'r+')

curr_pos = np.array([plot_pos_x, plot_pos_y]).transpose()
centroid_pos = np.array([centroid_x, centroid_y]).transpose()

fig_local = plt.figure("Local Map of Robot" + str(robot_id))
local_map = env.GetRobotLocalMap(robot_id)
local_ax = sns.heatmap(data=np.flip(local_map.transpose(),0), vmax=params_.pNorm, cmap=colormap, square=True)
cbar_ax = fig_local.axes[-1]# retrieve previous cbar_ax (if exists)

cont_flag = True
prev_robot_pos = robot_positions

ax.plot([curr_pos[0][0], centroid_pos[0][0]], [curr_pos[0][1], centroid_pos[0][1]], 'r')
for i in range(1, num_robots):
    ax.plot([curr_pos[i][0], centroid_pos[i][0]], [curr_pos[i][1], centroid_pos[i][1]])
plt.show()
while num_steps < params_.pEpisodeSteps:
    print(num_steps)
    cont_flag = oracle.Step()
    actions = oracle.GetActions()
    if(not cont_flag):
        break
    robot_positions = env.GetRobotPositions()
    local_features = env.GetLocalVoronoiFeatures()
    print(local_features[0])
    # pool = Pool(num_robots)
    # pool.map(write_npz, range(0, num_robots))
    num_steps = num_steps + 1

    for i in range(0, num_robots):
        plot_pos_x[i] =  prev_robot_pos[i][0] / params_.pResolution
        plot_pos_y[i] =  prev_robot_pos[i][1] / params_.pResolution

    local_map = env.GetRobotLocalMap(robot_id)
    sns.heatmap(ax=local_ax,data=np.flip(local_map.transpose(), 0), vmax=params_.pNorm, cmap=colormap, square=True, cbar_ax=cbar_ax, xticklabels=[],yticklabels=[])
    local_ax.set_title("Robot [" + str(robot_id) + "] position: " + "{:.{}f}".format(robot_positions[robot_id][0], 2) + ", " +  "{:.{}f}".format(robot_positions[robot_id][1], 2))

    prev_robot_pos = robot_positions
    plot_robots.set_xdata(plot_pos_x)
    plot_robots.set_ydata(plot_pos_y)
    fig.canvas.draw()
    fig.canvas.flush_events()
    fig_local.canvas.draw()
    fig_local.canvas.flush_events()

