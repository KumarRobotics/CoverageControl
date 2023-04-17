See the python script `generate_osm_map.py`.  
The function OverpassOSMQuery requires a params file, an origin, and a filename to save data.  
The origin is the bottom left corner of the bounding box.  
The semantic data is stored in the geoJSON format.  

The semantic data can be visualized using the web interface in `leaflet_geojson_viz` directory.  
Check semantic data in `leaflet_geojson_viz/data/semantic_data.json`. Create the `data` directory first else it gives an error while saving data   
Then launch a local server inside `leaflet_geojson_viz` directory using `python -m http.server`  
Open the link shown in the above step in a browser.

The OSM planet data can be hosted on a local server:
https://hub.docker.com/r/wiktorn/overpass-api

