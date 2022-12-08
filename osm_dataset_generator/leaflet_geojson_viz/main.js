/**
 *
 * @author Saurav Agarwal
 * @contact SauravAg@UPenn.edu
 * @contact agr.saurav1@gmail.com
 *
 */

var map = L.map('map').setView([39.9577545, -75.1883271], 16);
var map_data_json;
var overpass_query_string = 'highway~"^(motorway|motorway_link|trunk|trunk_link|primary|secondary|tertiary|unclassified|residential)$"';

L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=pk.eyJ1Ijoic2FnYXJ3MTAiLCJhIjoiY2t4MWU0eHprMWR4ZDJ1cW94ZjRkamM1bCJ9.b5dLgGYJuIxpIR5woCF8lw', {
	maxZoom: 20,
	attribution: '<a href="https://saurav.fyi">Saurav Agarwal</a> | Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, ' +
	'Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
	id: 'mapbox/streets-v11',
	tileSize: 512,
	zoomOffset: -1
}).addTo(map);



var geojsonMarkerOptions = {
	radius: 8,
	fillColor: "#d62728",
	color: "#000",
	weight: 2,
	opacity: 1,
	fillOpacity: 0.8,
	marker: 'circle'
};

stylefn = function(feature) {
	if(feature.properties.type == "road_network") {
		return { weight: 2 }
	}
	if(feature.properties.type == "traffic_signal") {
		return geojsonMarkerOptions;
	}
	if(feature.properties.type == "amenity") {
		switch(feature.properties.subtype) {
			case 'parking': return {fillColor: '#8c564b', color: 'black', weight: 2, fillOpacity: 0.6};
			case 'hospital': return {fillColor: '#1b4f72', color: 'black', weight: 2, fillOpacity: 0.6};
			case 'fuel': return {fillColor: '#9467bd', color: 'black', weight: 2, fillOpacity: 0.6};
		}
	}
}

var semantic_data;
fetch("data/semantic_data.json")
	.then(response => response.json())
	.then(data => {
		semantic_data = data;
		L.geoJson(data, {
			pointToLayer: function (feature, latlng) {
				return L.circleMarker(latlng, geojsonMarkerOptions);
			},
			style: stylefn
		}).addTo(map);
		// bbox = semantic_data.bbox;
		// L.rectangle([[bbox[1], bbox[0]], [bbox[3], bbox[2]]], {color: "#ff7800", weight: 1}).addTo(map);
		map.flyTo(semantic_data.center, 16);
	});
