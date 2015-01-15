export ROS_HOME=/media/q50_data/ros_data

rostopic pub /arduino/camera_gps_controller std_msgs/String "FREEZE_CAMERAS" --once
sleep 2s
roslaunch q50_launch start_recording.launch basename:=$1 & 
sleep 4s # wait for the recording software to start
rostopic pub /arduino/camera_gps_controller std_msgs/String "TRIGGER_CAMERAS" --once
#roslaunch novatel_serial_driver start_triggering.launch


