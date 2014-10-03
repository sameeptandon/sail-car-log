This works on both OSX and linux

Prerequisite: libusb 

On OSX, you could do
port install libusb

On Linux, you should install libusb-1.0.18 from source. 

Once that is done, open up the file src/test.cpp.

Edit the fields CAMERA_0_VID / CAMERA_0_PID and CAMERA_1_VID / CAMERA_1_PID based on the values from your camera. On OSX, you can find these by going to Apple -> About this Mac -> System Report -> USB. On linux, you can find these via the command lsusb. These fields tell the program which camera to load.

Next we'll compile the code. For this, do (at the top level of the libuvc directory):

mkdir build
cd build
cmake ..
make
./test

If all goes well, you should see some diagnostics information printed out for each camera, and eventually a display. On OSX, you might need to kill and run it 2 or 3 times before it shows up properly. On linux, it should work the first time.
