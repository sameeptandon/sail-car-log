export ROS_HOME=~/ros_data

roslaunch q50_launch start_recording.launch basename:=$1 & 
sleep 5s # wait for the recording software to start
roslaunch novatel_serial_driver start_triggering.launch


