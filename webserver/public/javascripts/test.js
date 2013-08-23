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

socket.on('INFOBUFFERSIZE', function(data) {
  document.getElementById('queue_size').innerHTML = 'Queue Size: ' + data;
});

socket.on('INFOCAPTURERATE', function(data) {
  document.getElementById('capture_rate').innerHTML = 'Capture Rate: ' + data;
});

socket.on('GPS', function(data) {
  document.getElementById('gps_pos').innerHTML = 'GPS Position: ' + data;
});

socket.on('disk_usage', function(data) {
  document.getElementById('disk_usage').innerHTML = 'Disk Usage: ' + data;
});

socket.on('subprocess_running', function(running) {
  var enabled = document.getElementById('start_button');
  var disabled = document.getElementById('stop_button');
  
  if (!running) {
    var tmp = enabled;
    enabled = disabled;
    disabled = enabled;
  }

  enabled.classList.remove('disabled');
  disabled.classList.add('disabled');
});

var sendStart = function() {
  var actives = $('.active');
  var minutes_before_reset = 10;

  if (actives.length > 0) {
    minutes_before_reset = parseInt(actives.attr('mins'), 10);
  }

  var data = {name: document.getElementById('filename_input').value, frames: 3000*minutes_before_reset};
  socket.emit('start_pressed', data);
};

var sendStop = function() {
  resetDisplay();

  socket.emit('stop_pressed');
};

var toggleDisabled = function() {
}

var resetDisplay = function() {
  if (warnExpirationTimeout) {
    clearTimeout(warnExpirationTimeout);
  }

  document.getElementById('warn_message').innerHTML = '';
  document.getElementById('capture_rate').innerHTML = '';
  document.getElementById('queue_size').innerHTML = '';
  document.getElementById('gps_pos').innerHTML = '';
}
