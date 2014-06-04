#include "ros/ros.h"
#include <trimble_gps/GpsState.h>
#include "trimble_gps.hpp"

// ip adress and port number of trimble GPS
#define SERVER  "169.254.1.0"
#define SERVPORT 5018

CGPSCom gps;
trimble_gps::GpsState msg;

 
int main(int argc, char ** argv)
{
  // ROS initialization
  ros::init(argc, argv, "trimble_gps_publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<trimble_gps::GpsState>("trimble_gps",10);
  ros::Rate loop_rate(20); // 20Hz
  ////////////////

  // GPS initialization
 
  char server[255];
  strcpy(server, SERVER);
  if(gps.Connect(server, SERVPORT))
    ROS_INFO("Trimble GPS: Connection Success");
  else {
    ROS_INFO("Trimble GPS: Connection Failure -> Exit");
    return 0;
  }    

  while(ros::ok())
    {
      // read GPS data
      if(!gps.ReceiveGPS()){
	ROS_INFO("Trimble GPS: Failed to read GPS data");
      }
      ROS_INFO("GPS Raw lat : %.10f  Raw lon: %.10f",gps.GPGGA.Latitude,gps.GPGGA.Longitude);
      msg.header.stamp = ros::Time().now();
      msg.UTC_Time = gps.GPGGA.UTC_Time;
      msg.Latitude = gps.GPGGA.Latitude;
      msg.NS_ind = gps.GPGGA.NS_ind;
      msg.Longitude = gps.GPGGA.Longitude;
      msg.EW_ind = gps.GPGGA.EW_ind;
      msg.Pos_Fix_ind = gps.GPGGA.Pos_Fix_ind;
      msg.Num_of_Satellite = gps.GPGGA.Num_of_Satellite;
      msg.HDOP = gps.GPGGA.HDOP;
      msg.Altitude = gps.GPGGA.Altitude;
      msg.Heading = gps.GPHDT.Heading;
      msg.TF_ind = gps.GPHDT.TF_ind;
      
      pub.publish(msg);

      ros::spinOnce();
      loop_rate.sleep();
    }

      
  return 0;
}
