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
def plot_map(map):
    ax = sns.heatmap(map.transpose(), vmax=255, cmap=colormap, square=True)
    ax.invert_yaxis()
    nrow, ncol = map.shape
    if(nrow > 50 and nrow < 500):
        septicks = 5 ** (math.floor(math.log(nrow, 5)) - 1)
    else:
        septicks = 10 ** (math.floor(math.log(nrow, 10)) - 1)
    plt.xticks(np.arange(0, nrow, septicks), np.arange(0, nrow, septicks))
    plt.yticks(np.arange(0, ncol, septicks), np.arange(0, ncol, septicks))
    plt.show()

params_ = pyCoverageControl.Parameters('parameters.yaml')

num_gaussians = 100
num_robots = 20
env = CoverageSystem(params_, num_gaussians, num_robots)
map = env.GetWorldIDF()
# plot_map(map)

voronoi_cells = env.GetVoronoiCells()
