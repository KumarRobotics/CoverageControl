import numpy as np
import math
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points
from pyCoverageControl import PointVector # for defining list of points

# We can visualize the map in python
import matplotlib.pylab as plt
import seaborn as sns
colormap = sns.color_palette("light:b", as_cmap=True)
def plot_map(map):
    ax = sns.heatmap(map.transpose(), cmap=colormap, square=True)
    ax.invert_yaxis()
    nrow, ncol = map.shape
    print(nrow)
    plt.xticks(np.arange(0, nrow, 100), np.arange(0, nrow, 100))
    plt.yticks(np.arange(0, ncol, 100), np.arange(0, ncol, 100))
    plt.show()


################
## Parameters ##
################
# The parameters can be changed from python without needing to recompile.
# The parameters are given in config/parameters.yaml
# After changing the parameters, use the following function call to use the yaml file.
# Make sure the path of the file is correct
params_ = pyCoverageControl.Parameters('parameters.yaml')

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

mean = Point2(10, 1000)
sigma = 5
peak_val = 3
dist2 = BND(mean, sigma, peak_val) # circular gaussian

mean3 = Point2(900,100)
sigma_skewed = Point2(2, 3)
rho = 2
dist3 = BND(mean3, sigma_skewed, rho, peak_val) # general BND

##############
## WorldIDF ##
##############
from pyCoverageControl import WorldIDF # for defining world idf
world_idf = WorldIDF(params_)
world_idf.AddNormalDistribution(dist1); # Add a distribution to the idf
world_idf.AddNormalDistribution(dist2); # Add a distribution to the idf
world_idf.AddNormalDistribution(dist3); # Add a distribution to the idf
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
