import generate_osm_map
import pickle
import yaml
import os

dataset_base_dir = '../../../data/osm_city_data/'
dataset_filename = 'km_region_origin_points'
path_to_dataset = dataset_base_dir + dataset_filename

params_filename = 'semantic_objects.yaml'

with open(params_filename, 'r') as file:
    params = yaml.safe_load(file)

with open(path_to_dataset, 'rb') as file:
    city_origin_points = pickle.load(file)

print("No. of points: ", len(city_origin_points))
count = 0
data_count = 0
for i in range(data_count, len(city_origin_points)):
    origin = city_origin_points[i]
    print(i, " lat: ", origin['lat'], " lon: ", origin['lon'])
    data_path = dataset_base_dir + str(count)
    semantic_data_filename = data_path + '/semantic_data.json'
    if not os.path.exists(data_path):
        os.makedirs(data_path)
    if(generate_osm_map.OverpassOSMQuery(params, origin, semantic_data_filename) == False):
        continue
    count = count + 1
    print(str(count))
