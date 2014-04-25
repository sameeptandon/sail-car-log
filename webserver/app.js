var DIAGNOSTICS_PORT = 5000
var CAMERALOGGER_PORT = 5001


/**
 * Module dependencies.
 */

var express = require('express');
var fs = require('fs');
var routes = require('./routes');
var user = require('./routes/user');
var http = require('http');
var path = require('path');
var spawn = require('child_process').spawn;
var util = require('./file_conventions.js');
var zmq = require('zmq');

var requester = zmq.socket('sub');
requester.bindSync("tcp://127.0.0.1:" + DIAGNOSTICS_PORT);
//requester.setsockopt(zmq.ZMQ_SUB, '');
requester.subscribe('');

var logger_socket = zmq.socket('pub');
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

var requested_terminate = false;

var server = http.createServer(app);
server.listen(app.get('port'), function(){
  console.log('Express server listening on port ' + app.get('port'));
});

var io = require('socket.io').listen(server);
io.sockets.on('connection', function(socket) {
  var running = subprocess ? true : false;
  socket.emit('subprocess_running', running);
  requester.on('message', function(res) {

    if (res.length >= 4 && res.slice(0, 4).toString() == 'CAM:') {
      fs.writeFile('public/images/test.png', res.slice(4), function(err) {
        if (err) throw err;
        socket.emit('update_image');
      });
    } else {
      var splits = res.toString().split(':');
      var header = splits[0];
      var msg = splits[1];

      socket.emit(header, msg);
    }
  });
  socket.on('start_pressed', function(data) {
    //console.log(data);
    if (subprocess) return;
    requested_terminate = false;
    spawnThread(data.name, data.frames);
  });

  socket.on('stop_pressed', function() {
    logger_socket.send('TERMINATE');
    requested_terminate = true;
  });

  checkDiskUsage(socket);
  setInterval(function() {
      checkDiskUsage(socket);
  }, 10000); // 10 seconds
});

var subprocess = null;

process.on('SIGINT', function() {
  logger_socket.send('TERMINATE');
  
  if (subprocess) {
    subprocess.on('exit', function() {
      process.exit(0);
    });
  } else {
    process.exit(0);
  }
});

var outdir = "/home/q50/ros_data/";

var spawnThread = function(prefix, maxFrames) {
  var name = prefix + '_' + util.getNextSuffix(outdir, prefix);
  var command = util.getCaptureCommand(name, maxFrames).split(' ');
  console.log(command);
  var head = command.splice(0, 1)[0];

  io.sockets.emit('subprocess_running', true);
  console.log("starting process");

  subprocess = spawn(head, command, {cwd: outdir, env: process.env});
  subprocess.stdout.on('data', function(data) {
    console.log(data.toString());
  });
  subprocess.stderr.on('data', function(data) {
    console.log('error: ' + data.toString());
  });
  subprocess.on('exit', function(code) {
    console.log(code);
    subprocess = null;
    if (code == 0 && !requested_terminate) {
        setTimeout(function() { 
            spawnThread(prefix, maxFrames);
        }, 1000);
    } else {
        setTimeout(function() { 
            io.sockets.emit('subprocess_running', false);
        }, 1000); 
    }

  });
}

var checkDiskUsage = function(socket) {
  var disk_check = spawn('df', ['-h']);
  var grep = spawn('grep', ['/dev/sda1']);
  var awk = spawn('awk', ['{print $5}']);

  disk_check.stdout.on('data', function(data) {
    grep.stdin.write(data);
  });

  disk_check.on('close', function(code) {
    grep.stdin.end();
  });

  grep.stdout.on('data', function(data) {
    awk.stdin.write(data);
  });

  grep.on('close', function(code) {
    awk.stdin.end();
  });

  awk.stdout.on('data', function(data) {
    var output = data.toString().trim();
    socket.emit('disk_usage', output);
  });
}
