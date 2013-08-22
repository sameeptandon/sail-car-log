socket = io.connect('/');

socket.on('button_response', function(res) {
  var diag_output = document.getElementById('diagnostics');
  var text_out = document.createElement('div');
  text_out.innerHTML = res;
  diag_output.insertBefore(text_out, diag_output.firstChild);
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
  socket.emit('stop_pressed');
};
