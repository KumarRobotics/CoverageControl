See the python script osm_semantic_map.py.
The function OverpassOSMQuery requires a params file, an origin, and a filename to save data.
The origin is the bottom left corner of the bounding box.
The semantic data is stored in the geoJSON format.

The semantic data can be visualized using the web interface in leaflet_geojson_viz directory.
Copy the semantic data as leaflet_geojson_viz/data/semantic_data.json
Then launch a local server inside leaflet_geojson_viz directory using python -m http.server
Open the link shown in the above step in a browser.

