socket = io.connect('/');

console.log('socket');
socket.on('button_response', function(res) {
  var diag_output = document.getElementById('diagnostics');
  var text_out = document.createElement('div');
  text_out.innerHTML = res;
  diag_output.insertBefore(text_out, diag_output.firstChild);
});

var sendStart = function() {
  socket.emit('start_pressed', document.getElementById('filename_input').value);
}

var sendStop = function() {
  socket.emit('stop_pressed', document.getElementById('filename_input').value);
}
