// Set up Google Maps

var map;
var numTracks = 0;
// http://en.wikipedia.org/wiki/Web_colors
var strokeColors = [
    '#ff0000', // red
    '#00ff00', // lime
    '#0000ff', // blue
    '#800000', // maroon
    '#000000', // black
    '#800080', // purple
    '#ffff00', // yellow
    '#808000', // olive
    '#ffffff', // white
    '#00ffff', // aqua

];
var paths = [];
var tracksActive = [];

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
            addTrackPanel(key);
            tracksActive.push(true);
            numTracks++;
        });
    });
}

function addTrackPanel(name) {
    $('#trackPanels').append('<div class="trackPanel"><input type="checkbox" checked="true" onclick="toggleTrack(' + numTracks + ')" /><span style="color:' + strokeColors[numTracks] + '">' + name + '</span></div>');
}

function toggleTrack(ind) {
    if (tracksActive[ind]) {
        paths[ind].setMap(null);
        tracksActive[ind] = false;
    }
    else {
        paths[ind].setMap(map);
        tracksActive[ind] = true;
    }
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
    paths.push(path);
}

// Onload

$(document).ready(function() {
    initializeMap();
    loadGPSTracks('gps_tracks.json');
});
