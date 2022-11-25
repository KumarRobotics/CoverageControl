import numpy as np
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points

# Point2 has two atrributes x and y
pt = Point2(0,0) # Needs two arguments for x and y
print(pt)
pt.x = 5
pt.y = 10
print(pt.x)


from pyCoverageControl import BivariateNormalDistribution as BND # for defining bivariate normal distributions
dist1 = BND() # zero-mean, sigma = 1, circular

mean = Point2(512, 512)
sigma = 5
peak_val = 3
dist2 = BND(mean, sigma, peak_val) # circular gaussian

sigma_skewed = Point2(1, 2)
rho = 2
dist3 = BND(mean, sigma_skewed, rho, peak_val) # general BND

from pyCoverageControl import WorldIDF # for defining world idf
world_idf = WorldIDF()
world_idf.AddNormalDistribution(dist2); # Add a distribution to the idf
world_idf.GenerateMapCuda(); # Generate map, use GenerateMap() for cpu version
# world_idf.WriteWorldMap("map.dat"); # Writes the matrix to the file. Map should have been generated before writing
map = world_idf.GetWorldMap(); # Get the world map as numpy nd-array. You can only "view", i.e., flags.writeable = False, flags.owndata = False
max_val = world_idf.GetMaxValue(); # Get the maximum importance value for the entire map. Useful for normalization
print(map.dtype)
print(map.flags)
print(type(map))
print(map[0, 0])
# We can visualize the map in python
# Doesn't look great as there aren't many distributions
# The axes are interchanged and y is flipped. How to fix this?
# Make tick positions multiples of 100
import matplotlib.pyplot as plt
import seaborn as sns
colormap = sns.color_palette("light:b", as_cmap=True)
ax = sns.heatmap(map, cmap=colormap)
ax.set_box_aspect(1)
plt.show()


