import numpy as np
import math
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points
from pyCoverageControl import PointVector # for defining list of points
from pyCoverageControl import CoverageSystem

# We can visualize the map in python
import matplotlib.pylab as plt
import seaborn as sns
colormap = sns.color_palette("light:b", as_cmap=True)
def plot_map(params, map, plot_pos_x, plot_pos_y, voronoi_cells):
    ax = sns.heatmap(map.transpose(), vmax=params.pNorm, cmap=colormap, square=True)
    ax.invert_yaxis()
    nrow, ncol = map.shape
    if(nrow > 50 and nrow < 500):
        septicks = 5 ** (math.floor(math.log(nrow, 5)) - 1)
    else:
        septicks = 10 ** (math.floor(math.log(nrow, 10)) - 1)
    plt.xticks(np.arange(0, nrow, septicks), np.arange(0, nrow, septicks))
    plt.yticks(np.arange(0, ncol, septicks), np.arange(0, ncol, septicks))
    ax.plot(plot_pos_x, plot_pos_y, 'go')
    centroid_x = np.array([])
    centroid_y = np.array([])
    for vcell in voronoi_cells:
        cell = vcell.cell
        plot_pt_x = np.array([])
        plot_pt_y = np.array([])
        for pt in cell:
            plot_pt_x = np.append(plot_pt_x, pt.x / params_.pResolution)
            plot_pt_y = np.append(plot_pt_y, pt.y / params_.pResolution)
        plot_pt_x = np.append(plot_pt_x, plot_pt_x[0])
        plot_pt_y = np.append(plot_pt_y, plot_pt_y[0])
        ax.plot(plot_pt_x, plot_pt_y, 'r')
        centroid_x = np.append(centroid_x, vcell.centroid.x)
        centroid_y = np.append(centroid_y, vcell.centroid.y)
    # for edge in voronoi_edges:
    #     ax.plot([edge.x1, edge.x2], [edge.y1, edge.y2], 'r')
    ax.plot(centroid_x, centroid_y, 'r+')
    plt.show()

params_ = pyCoverageControl.Parameters('parameters.yaml')

num_gaussians = 100
num_robots = 40
env = CoverageSystem(params_, num_gaussians, num_robots)
map = env.GetWorldIDF()
robot_positions = env.GetRobotPositions()
print(type(map))
print(type(robot_positions))
plot_pos_x = np.array([])
plot_pos_y = np.array([])
for pos in robot_positions:
    plot_pos_x = np.append(plot_pos_x, pos.x / params_.pResolution)
    plot_pos_y = np.append(plot_pos_y, pos.y / params_.pResolution)

env.ComputeVoronoiCells()
voronoi_cells = env.GetVoronoiCells()
voronoi_edges = env.GetVoronoiEdges()
plot_map(params_, map, plot_pos_x, plot_pos_y, voronoi_cells)
