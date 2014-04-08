var path = require('path');
var fs = require('fs');

module.exports = {
  getNextSuffix: function(dir, prefix) {
    files = fs.readdirSync(dir);
    gps_logs = files.filter(function(elem) {
      return elem.indexOf('gps.bag') != -1 && elem.indexOf(prefix) != -1;
    });
    return String.fromCharCode('a'.charCodeAt(0) + gps_logs.length);
  },

  getCaptureCommand: function(name, maxFrames) {
    var cmd = 'rosrun q50_tablet_backend q50_tablet_backend.py _basename:=' + name;
    if (maxFrames) {
      cmd = cmd + ' _maxframes:=' + maxFrames
    }

    return cmd
  }
/*
  getCaptureCommand: function(name, maxFrames) {
    var cmd = 'sudo /home/smart/sail-car-log/cameralogger/build/CameraLogger -s /dev/serial/by-id/usb-09d7_0210-if00 -l -o ' + name;
    if (maxFrames) {
      cmd = cmd + ' -m ' + maxFrames
    }

    return cmd
  }
*/
}
