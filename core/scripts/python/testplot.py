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
params_ = pyCoverageControl.Parameters('params/parameters.yaml')

############
## Point2 ##
############
# Point2 has two atrributes x and y
pt = Point2(0,0) # Needs two arguments for x and y
print(pt)
pt[0] = 5
pt[1] = 10
print(pt[0])

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
sigma = 20
peak_val = 200
dist2 = BND(mean, sigma, peak_val) # circular gaussian

mean3 = Point2(900,100)
sigma_skewed = Point2(2, 3)
rho = 0.5
dist3 = BND(mean3, sigma_skewed, rho, peak_val) # general BND

##############
## WorldIDF ##
##############
from pyCoverageControl import WorldIDF # for defining world idf
world_idf = WorldIDF(params_)
world_idf.AddNormalDistribution(dist1); # Add a distribution to the idf
world_idf.AddNormalDistribution(dist2); # Add a distribution to the idf
# world_idf.AddNormalDistribution(dist3); # Add a distribution to the idf
print("Calling CUDA")
world_idf.GenerateMapCuda() # Generate map, use GenerateMap() for cpu version
world_idf.PrintMapSize()
# world_idf.WriteWorldMap("map.dat"); # Writes the matrix to the file. Map should have been generated before writing
map = world_idf.GetWorldMap(); # Get the world map as numpy nd-array. You can only "view", i.e., flags.writeable = False, flags.owndata = False
normalization_factor = world_idf.GetNormalizationFactor();
print(map.dtype)
print(map.flags)
print(type(map))
print(map[0, 0])
plot_map(map)
