// Set up Google Maps

var map;
var numTracks = 0;
var strokeColors = [
    '#ff0000',
    '#00ff00',
    '#0000ff'
];

function initializeMap() {
    var mapOptions = {
      center: new google.maps.LatLng(lat_center, lon_center),
      zoom: 8
    };
    map = new google.maps.Map(document.getElementById("map-canvas"), mapOptions);
}
//google.maps.event.addDomListener(window, 'load', initialize);

// Load GPS

function loadGPSTracks(f) {
    $.getJSON(f, function(data) {
        $.each(data, function(key, val)
        {
            console.log(key);
            console.log(val['lat'].length)
            var lat = val['lat'];
            var lon = val['lon'];
            drawGPSTrack(lat, lon);
            numTracks++;
        });
    });
}

function drawGPSTrack(lat, lon) {
    var coords = [];
    $.each(lat, function(ind, val) {
        coords.push(new google.maps.LatLng(val, lon[ind]));
    });

    var path = new google.maps.Polyline({
        path: coords,
        geodesic: true,
        strokeColor: strokeColors[numTracks],
        strokeOpacity: 0.7,
        strokeWeight: 3
    });
    path.setMap(map);
}

// Onload

$(document).ready(function() {
    initializeMap();
    loadGPSTracks('gps_tracks.json');
});
