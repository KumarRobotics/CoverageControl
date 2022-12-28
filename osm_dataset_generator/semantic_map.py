import numpy as np
import math
import pyCoverageControl # Main library
from pyCoverageControl import BivariateNormalDistribution as BND # for defining bivariate normal distributions
from pyCoverageControl import GeoLocalTransform as GeoTransform

import json
import geojson

# We can visualize the map in python
import matplotlib.pylab as plt
import seaborn as sns
colormap = sns.color_palette("light:b", as_cmap=True)
def plot_map(map):
    ax = sns.heatmap(map.transpose(), vmax=1, cmap=colormap, square=True)
    ax.invert_yaxis()
    nrow, ncol = map.shape
    septicks = 10 ** (math.floor(math.log(nrow, 10)) - 1)
    plt.xticks(np.arange(0, nrow, septicks), np.arange(0, nrow, septicks))
    plt.yticks(np.arange(0, ncol, septicks), np.arange(0, ncol, septicks))
    plt.show()


################
## Parameters ##
################
# The parameters can be changed from python without needing to recompile.
# The parameters are given in params/parameters.yaml
# After changing the parameters, use the following function call to use the yaml file.
# Make sure the path of the file is correct
params_ = pyCoverageControl.Parameters('../core/params/parameters.yaml')

##############
## WorldIDF ##
##############
from pyCoverageControl import WorldIDF # for defining world idf
world_idf = WorldIDF(params_)

with open("leaflet_geojson_viz/data/semantic_data.json") as file_:
    semantic_data = geojson.load(file_)

[origin_lon, origin_lat] = semantic_data.bbox[0:2]
origin_alt = 0
geo_transform = GeoTransform(origin_lat, origin_lon, origin_alt)

# BivariateNormalDistribution with peak value of 1
traffic_signals_sigma = 10
traffic_signals_scale = 2 * math.pi * traffic_signals_sigma * traffic_signals_sigma

# Uniform density polygon
hostpital_importance = 0.9
parking_importance = 0.7

flag = True
for feature in semantic_data.features:
    if(feature['properties']['type'] == "traffic_signal"):
        coords = feature['geometry']['coordinates']
        mean = geo_transform.Forward(coords[1], coords[0], origin_alt)
        dist = BND(mean[0:2], traffic_signals_sigma, traffic_signals_scale) # circular gaussian
        world_idf.AddNormalDistribution(dist)
    if(feature['properties']['type'] == "amenity" and feature['geometry']['type'] == "Polygon"):
        for coords_list in feature['geometry']['coordinates']:
            polygon_feature = pyCoverageControl.PolygonFeature()
            if(feature['properties']['subtype'] == "hospital"):
                polygon_feature.imp = hostpital_importance
            if(feature['properties']['subtype'] == "parking"):
                polygon_feature.imp = parking_importance
            for pt in coords_list[:-1]:
                cartesian_pt = geo_transform.Forward(pt[1], pt[0], origin_alt)
                polygon_feature.poly.append(cartesian_pt[0:2])
            world_idf.AddUniformDistributionPolygon(polygon_feature)


world_idf.GenerateMapCuda() # Generate map, use GenerateMap() for cpu version
world_idf.PrintMapSize()
# world_idf.WriteWorldMap("map.dat"); # Writes the matrix to the file. Map should have been generated before writing
map = world_idf.GetWorldMap(); # Get the world map as numpy nd-array. You can only "view", i.e., flags.writeable = False, flags.owndata = False
plot_map(map)
