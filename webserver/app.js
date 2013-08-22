var DIAGNOSTICS_PORT = 5000
var CAMERALOGGER_PORT = 5001


/**
 * Module dependencies.
 */

var express = require('express');
var routes = require('./routes');
var user = require('./routes/user');
var http = require('http');
var path = require('path');
var spawn = require('child_process').spawn;
var util = require('./file_conventions.js');
var zmq = require('zmq');

var requester = zmq.socket('req');
requester.connect("tcp://127.0.0.1:" + DIAGNOSTICS_PORT);

var logger_socket = zmq.socket('req');
logger_socket.connect('tcp://localhost:' + CAMERALOGGER_PORT);

var app = express();

// all environments
app.set('port', process.env.PORT || 3000);
app.set('views', __dirname + '/views');
app.set('view engine', 'jade');
app.use(express.favicon());
app.use(express.logger('dev'));
app.use(express.bodyParser());
app.use(express.methodOverride());
app.use(app.router);
app.use(express.static(path.join(__dirname, 'public')));

// development only
if ('development' == app.get('env')) {
  app.use(express.errorHandler());
}

app.get('/', routes.index);
app.get('/users', user.list);

var server = http.createServer(app);
server.listen(app.get('port'), function(){
  console.log('Express server listening on port ' + app.get('port'));
});

var io = require('socket.io').listen(server);
io.sockets.on('connection', function(socket) {
  requester.on('message', function(res) {
    socket.emit('button_response', res.toString());
  });
  socket.on('start_pressed', function(name) {
    requester.send(name);
  });
});

var subprocess = null;

var maxFrames = null;
if (process.argv.length > 3) {
  maxFrames = parseInt(process.argv[3], 10);
}

process.on('SIGINT', function() {
  logger_socket.send('TERMINATE');
  
  if (subprocess) {
    subprocess.on('exit', function() {
      process.exit(0);
    });
  }
});

var spawnThread = function(prefix, maxFrames) {
  var name = prefix + '_' + util.getNextSuffix(prefix);
  var command = util.getCaptureCommand(name, maxFrames).split(' ');
  var head = command.splice(0, 1)[0];

  //subprocess = spawn(head, command, {cwd: process.cwd(), env: process.env});
  subprocess.on('exit', function(code) {
    if (code == 1) {
      process.exit(0);
    } else {
      spawnThread(prefix, maxFrames);
    }
  });
}

spawnThread(process.argv[2])
