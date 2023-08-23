import generate_osm_map
import random
import pickle
import yaml
import os
import csv
import numpy as np
import math
import pyCoverageControl # Main library
from pyCoverageControl import BivariateNormalDistribution as BND # for defining bivariate normal distributions
from pyCoverageControl import GeoLocalTransform as GeoTransform
from pyCoverageControl import WorldIDF # for defining world idf
import json
import geojson
from pyCoverageControl import CoverageSystem
from pyCoverageControl import Point2 # for defining points
from pyCoverageControl import PointVector # for defining list of points


dataset_base_dir = '/root/CoverageControl_ws/datasets/50_cities/'
dataset_filename = 'cities_origins'

params_filename = dataset_base_dir + 'semantic_objects.yaml'
path_to_dataset = dataset_base_dir + dataset_filename

with open(params_filename, 'r') as file:
    params = yaml.safe_load(file)

with open(path_to_dataset, 'r') as file:
    csv_reader = csv.reader(file, delimiter='\t')
    city_origin_points = []
    city_names = []
    for row in csv_reader:
        city_origin_points.append({'lat': float(row[1]), 'lon': float(row[2])})
        city_names.append(row[0])

params_ = pyCoverageControl.Parameters(dataset_base_dir + '/env_params.yaml')
count = 0
data_count = 0
no_data_count = 0
for i in range(data_count, len(city_origin_points)):
# for i in range(data_count, 5):
    origin = city_origin_points[i]
    print(city_names[i])
    # print(origin)
    data_path = dataset_base_dir + city_names[i]
    semantic_data_filename = data_path + '/semantic_data.json'
    print(semantic_data_filename)
    if not os.path.exists(data_path):
        os.makedirs(data_path)
    if(generate_osm_map.OverpassOSMQuery(params, origin, semantic_data_filename) == False):
        no_data_count = no_data_count + 1
        print('=====================No data for ' + city_names[i])
    count = count + 1
    world_idf = WorldIDF(params_)
    with open(semantic_data_filename) as file_:
        semantic_data = geojson.load(file_)

    [origin_lon, origin_lat] = semantic_data.bbox[0:2]
    origin_alt = 0
    geo_transform = GeoTransform(origin_lat, origin_lon, origin_alt)

    for feature in semantic_data.features:
        if(feature['properties']['type'] == "traffic_signal"):
            coords = feature['geometry']['coordinates']
            mean = geo_transform.Forward(coords[1], coords[0], origin_alt)

            traffic_signals_sigma = random.uniform(params_.pMinSigma, params_.pMaxSigma)
            traffic_signals_scale = random.uniform(params_.pMinPeak, params_.pMaxPeak)

            dist = BND(mean[0:2], traffic_signals_sigma, traffic_signals_scale) # circular gaussian
            world_idf.AddNormalDistribution(dist)

    world_idf.GenerateMapCuda()
    robot_positions = PointVector()
    for i in range(params_.pNumRobots):
        robot_positions.append(np.array([random.uniform(0, params_.pRobotInitDist), random.uniform(0, params_.pRobotInitDist)]))
    env = CoverageSystem(params_, world_idf, robot_positions)
    env.WriteEnvironment(data_path + '/pos.dat', data_path + '/idf.dat')
    env.PlotWorldMap(data_path, 'idf')

print('Total cities: ' + str(count))
print('Cities with no data: ' + str(no_data_count))
