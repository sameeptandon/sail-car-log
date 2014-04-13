var warnExpirationTimeout = null;

socket = io.connect('/');

socket.on('update_image', function() {
  document.getElementById('gps_img').src = '/images/test.png?' + Date.now();
});

socket.on('WARN', function(data) {
  if (warnExpirationTimeout) {
    clearTimeout(warnExpirationTimeout);
  }

  var warnDiv = document.getElementById('warn_message');
  warnDiv.innerHTML = data;

  warnExpirationTimeout = setTimeout(function() {
    warnDiv.innerHTML = '';
  }, 3000);
});

socket.on('GPSUNCERTAINTY', function(data) {
  document.getElementById('gps_uncertainty').innerHTML = 'Err: ' + data;
});

socket.on('INFOCAPTURERATE', function(data) {
  document.getElementById('capture_rate').innerHTML = 'Capture Rate: ' + data;
});

socket.on('GPS', function(data) {
  var coords = data.split(',');
  coords[0] = coords[0].slice(0, coords[0].indexOf('.') + 4);
  coords[1] = coords[1].slice(0, coords[1].indexOf('.') + 4);
  data = coords[0] + ', ' + coords[1];
  document.getElementById('gps_pos').innerHTML = 'GPS Position: ' + data;
});

socket.on('disk_usage', function(data) {
  document.getElementById('disk_usage').innerHTML = 'Disk Usage: ' + data;
});

socket.on('subprocess_running', function(running) {
  var enabled = document.getElementById('start_button');
  var disabled = document.getElementById('stop_button');
  
  console.log(running);
  if (running) {
    enabled = document.getElementById('stop_button');
    disabled = document.getElementById('start_button');
  }

  console.log(enabled);
  console.log(disabled);
  enabled.classList.remove('disabled');
  disabled.classList.add('disabled');
});

var sendStart = function() {
  var name = document.getElementById('filename_input').value;
  var warnDiv = document.getElementById('warn_message');
  if (name == '') {
    warnDiv.innerHTML = 'Please input a name before hitting start';
    return;
  } else {
    warnDiv.innerHTML = '';
  }

  var actives = $('.active');
  var minutes_before_reset = 10;

  if (actives.length > 0) {
    minutes_before_reset = parseInt(actives.attr('mins'), 10);
  }

  var data = {name: name, frames: 3000*minutes_before_reset};
  socket.emit('start_pressed', data);
};

var sendStop = function() {
  resetDisplay();

  socket.emit('stop_pressed');
};

var resetDisplay = function() {
  if (warnExpirationTimeout) {
    clearTimeout(warnExpirationTimeout);
  }

  console.log('woeifjioj');
  document.getElementById('warn_message').innerHTML = '';
  document.getElementById('capture_rate').innerHTML = '';
  document.getElementById('gps_uncertainty').innerHTML = '';
  document.getElementById('gps_pos').innerHTML = '';
}
