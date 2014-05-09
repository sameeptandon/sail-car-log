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

var allLat = [];
var allLon = [];
var heatmap;

function initializeMap() {
    var mapOptions = {
      center: new google.maps.LatLng(lat_center, lon_center),
      zoom: 8
    };
    map = new google.maps.Map(document.getElementById("map-canvas"), mapOptions);
}
//google.maps.event.addDomListener(window, 'load', initialize);

function drawHeatmap() {
    console.log('Drawing heatmap ' + allLat.length);
    var coords = [];
    $.each(allLat, function(ind, val) {
        coords.push(new google.maps.LatLng(val, allLon[ind]));
    });
    var coordArray = new google.maps.MVCArray(coords);
    heatmap = new google.maps.visualization.HeatmapLayer({
        data: coordArray
    });
    heatmap.setMap(map);
}

function toggleHeatmap() {
    if (typeof heatmap == 'undefined')
        drawHeatmap()
    else
        heatmap.setMap(heatmap.getMap() ? null : map);
}

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
                    // NOTE push.apply(a, b) could fail for long b?
                    allLat.push.apply(allLat, lat);
                    allLon.push.apply(allLon, lon);
                    drawGPSTrack(lat, lon, [route, segment, split]);
                    segmentDiv.append('<div class="trackPanel"><input type="checkbox" checked="true" onchange="toggleTrack(' + numTracks + ')" /><span style="color:' + strokeColors[numRoutes % strokeColors.length] + '">' + split + '</span></div>');
                    numTracks++;
                });
                routeDiv.append(segmentDiv);
            });
            addTrackPanel(routeDiv);
            numRoutes++;
        });

        //drawHeatmap();

        $('#checkAll').click(function() {
            if ($(this).html() == 'Check All') {
                $('input:checkbox').prop('checked', 'checked');
                $(this).html('Uncheck All');
            } else {
                $('input:checkbox').removeProp('checked');
                $(this).html('Check All');
            }
            $('input:checkbox').trigger('change');
        });

    });
}

function addTrackPanel(routeDiv) {
    $('#trackPanels').append(routeDiv);
}

function toggleTrack(ind) {
    paths[ind].setMap(paths[ind].getMap() ? null: map);
}

function drawGPSTrack(lat, lon, pathInfo) {
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

    google.maps.event.addDomListener(path, 'click', function() {
        $('#pathInfo').html(pathInfo.join(' / '));
    });
}

// Onload

$(document).ready(function() {
    initializeMap();
    loadGPSTracks('gps_tracks.json');

});
