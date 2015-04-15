#roslaunch novatel_serial_driver stop_triggering.launch &
rostopic pub /arduino/camera_gps_controller std_msgs/String "FREEZE_CAMERAS" --once
sleep 2s # wait for ROS buffers to clean up 
#rosnode kill /fwd_left_writer /fwd_right_writer /wfov_front_writer /wfov_left_writer /wfov_right_writer /wfov_back_writer /rosbag_record_gps /rosbag_record_odb /rosbag_record_radar /tcpdump/velodyne_node /novatel_config_m2
rosnode kill /ov_601_writer /ov_602_writer /ov_603_writer /ov_604_writer /rosbag_record_gps /rosbag_record_odb /rosbag_record_radar /rosbag_record_mobileye /tcpdump/velodyne_node
#rosnode kill /fwd_left_writer /fwd_right_writer /rosbag_record_gps /rosbag_record_odb /rosbag_record_radar /tcpdump/velodyne_node /novatel_config

rostopic pub /arduino/camera_gps_controller std_msgs/String "TRIGGER_CAMERAS" --once
