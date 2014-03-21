#include "ros/ros.h"
#include <nodelet/loader.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "pointgrey_camera_node");
  
  // This is code based nodelet loading, the preferred nodelet launching is done through roslaunch
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  nodelet.load("pointgrey_camera_node", "pointgrey_camera_driver/PointGreyCameraNodelet", remap, nargv);
  
  ros::spin();

  return 0;
}
