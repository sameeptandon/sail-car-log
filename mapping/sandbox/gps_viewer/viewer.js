// Set up Google Maps

var map;
var numTracks = 0;
var numRoutes = 0;
// http://en.wikipedia.org/wiki/Web_colors
var strokeColors = [
    '#ff0000', // red
    '#00ff00', // lime
    '#0000ff', // blue
    '#800000', // maroon
    '#000000', // black
    '#800080', // purple
    '#ff00ff', // fuchsia
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
    $.getJSON(f, function(route_segment_split_gps) {
        $.each(route_segment_split_gps, function(route, segment_split_gps) {
            routeDiv = $('<div class="routeDiv">' + route + '</div>')
            console.log(route);
            $.each(segment_split_gps, function (segment, split_gps) {
                console.log('\t' + segment);
                segmentDiv = $('<div class="segmentDiv">' + segment + '</div>')
                $.each(split_gps, function(split, gps) {
                    if (!('lat' in gps))
                        return;
                    console.log('\t\t' + split + ': ' + gps['lat'].length);
                    var lat = gps['lat'];
                    var lon = gps['lon'];
                    drawGPSTrack(lat, lon);
                    segmentDiv.append('<div class="trackPanel"><input type="checkbox" checked="true" onclick="toggleTrack(' + numTracks + ')" /><span style="color:' + strokeColors[numRoutes % strokeColors.length] + '">' + split + '</span></div>');
                    tracksActive.push(true);
                    numTracks++;
                });
                routeDiv.append(segmentDiv);
            });
            addTrackPanel(routeDiv);
            numRoutes++;
        });
    });
}

function addTrackPanel(routeDiv) {
    $('#trackPanels').append(routeDiv);
    //$('#trackPanels').append('<div class="trackPanel"><input type="checkbox" checked="true" onclick="toggleTrack(' + numTracks + ')" /><span style="color:' + strokeColors[numRoutes % strokeColors.length] + '">' + name + '</span></div>');
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
        strokeColor: strokeColors[numRoutes],
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
