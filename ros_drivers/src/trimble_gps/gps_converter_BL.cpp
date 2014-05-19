


#include "ros/ros.h"
#include "car_msgs/GpsState.h"
#include "math.h"
#define _USE_MATH_DEFINES

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "Wgs2Htz.h"

#define Deg2Rad	 M_PI/180.0

// ROS initialization
ros::Publisher pose_pub;

void Callback(const car_msgs::GpsState& msg) {
  //void Callback(const car_msgs::GpsState& msg, ros::Publisher& pose_pub) {
  ////////////////

  CWgs2Htz wgs2htz;

  double x, y;
  double psi, lambda;
  double roll = 0;
  double pitch = 0;
  double yaw = 0;

  /////////////////////
  // publish pose message
  psi = msg.Latitude;
  lambda = msg.Longitude;
  if(!msg.EW_ind) { lambda = -lambda; }
//	ROS_INFO("East?; %d", gps.GPGGA.EW_ind);
  
  wgs2htz.ToHtz(psi, lambda, x, y);

  //  ROS_INFO("x: %f  y: %f",x, y);

  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.pose.pose.position.x = x -20935164;
  pose_msg.pose.pose.position.y = -y -1132072;
  pose_msg.pose.pose.position.z = 0.0 ;
  yaw = (360-msg.Heading) /180.0 * M_PI;

  pose_msg.pose.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
  pose_msg.header.frame_id = "world";
  pose_msg.header.stamp = ros::Time::now();
  //  pose_msg.header.stamp = msg.header.stamp;
  
  pose_pub.publish(pose_msg);  

// for testing
//  double psi_t=37.410379;
//  double lambda_t=-122.023683;
  //  double psi_t=36.5;
  //  double lambda_t=-120.5;
  //  double x_t, y_t;
  //  wgs2htz.ToHtz(psi_t, lambda_t, x_t, y_t);
  //  ROS_INFO("x: %f  y: %f",x_t,y_t);
   
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "gps_converter");
  ros::NodeHandle n;
  pose_pub= n.advertise<geometry_msgs::PoseWithCovarianceStamped>("Pose_gps", 100);

  //  ros::Subscriber sub = n.subscribe<car_msgs::GpsState>("trimble_gps", 100, boost::bind(Callback, _1, pose_pub));
  ros::Subscriber sub = n.subscribe("trimble_gps", 100, Callback);
  ros::spin();
      
  return 0;
}