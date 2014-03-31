roslaunch novatel_serial_driver stop_triggering.launch &
sleep 5s # wait for ROS buffers to clean up 
rosnode kill /fwd_left_writer /fwd_right_writer /rosbag_record_gps /rosbag_record_odb /rosbag_record_radar /tcpdump/velodyne_node /novatel_config

