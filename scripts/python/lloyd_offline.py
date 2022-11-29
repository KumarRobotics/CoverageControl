import sys
import math
import time
import numpy as np
import sklearn
from sklearn.metrics import pairwise_distances as pwdist
from scipy.optimize import linear_sum_assignment
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
num_robots = 40
env = CoverageSystem(params_, num_gaussians, num_robots)
map = env.GetWorldIDF()
robot_positions = env.GetRobotPositions()
print(type(map))
print(type(robot_positions))

voronoi_cells = env.LloydOffline()
print("LloydOffline completed")



fig = plt.figure("Environment")
ax = sns.heatmap(map.transpose(), vmax=params_.pNorm, cmap=colormap, square=True)
ax.invert_yaxis()
nrow, ncol = map.shape
# septicks = 5 ** (math.floor(math.log(nrow, 5)) - 1)
septicks = 10 ** (math.floor(math.log(nrow, 10)))
plt.xticks(np.arange(0, nrow, septicks), np.arange(0, nrow, septicks))
plt.yticks(np.arange(0, ncol, septicks), np.arange(0, ncol, septicks))

plot_pos_x = np.array([])
plot_pos_y = np.array([])
for pos in robot_positions:
    plot_pos_x = np.append(plot_pos_x, pos.x / params_.pResolution)
    plot_pos_y = np.append(plot_pos_y, pos.y / params_.pResolution)

plot_robots, = ax.plot(plot_pos_x, plot_pos_y, 'go')
print(plot_robots)
centroid_x = np.array([])
centroid_y = np.array([])
plot_cells = []
for vcell in voronoi_cells:
    cell = vcell.cell
    plot_pt_x = np.array([])
    plot_pt_y = np.array([])
    for pt in cell:
        plot_pt_x = np.append(plot_pt_x, pt.x / params_.pResolution)
        plot_pt_y = np.append(plot_pt_y, pt.y / params_.pResolution)
    plot_pt_x = np.append(plot_pt_x, plot_pt_x[0])
    plot_pt_y = np.append(plot_pt_y, plot_pt_y[0])
    plot_cell, = ax.plot(plot_pt_x, plot_pt_y, 'r')
    plot_cells.append(plot_cell)
    centroid_x = np.append(centroid_x, vcell.centroid.x)
    centroid_y = np.append(centroid_y, vcell.centroid.y)
plot_centroids, = ax.plot(centroid_x, centroid_y, 'r+')

curr_pos = np.array([plot_pos_x, plot_pos_y]).transpose()
centroid_pos = np.array([centroid_x, centroid_y]).transpose()
cost_matrix = pwdist(curr_pos, centroid_pos)
row_ind, col_ind = linear_sum_assignment(cost_matrix)
for i in range(0, len(col_ind)):
    ax.plot([curr_pos[i][0], centroid_pos[col_ind[i]][0]], [curr_pos[i][1], centroid_pos[col_ind[i]][1]])
plt.show()



