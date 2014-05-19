#include "ros/ros.h"
#include <trimble_gps/GpsState.h>
#include "math.h"
#define _USE_MATH_DEFINES

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "Wgs2Htz.h"

#define Deg2Rad	 M_PI/180.0

// ROS initialization
ros::Publisher pose_pub;
CWgs2Htz wgs2htz;


double convert(double value) {
  double deg = (int)value/100; 
  double min =  value - deg*100;
  double result = deg + min/60.0;
  
  return result;  
}


void convertToLocalMap(double x, double y, double & xNew, double & yNew) {
  // X0 and YO Are dependent on the the local base
  double X0 = 12.3;
  double Y0 = -13.07;
  double theta0 = wgs2htz.GetLocalZeroHeading();//14.869087;// 14.7715488;//1.884009;//14.7715488 ;//degrees
  
  //double theta0 = 0;//14.7715488;//degrees
  //change the value above if you change the reference base
  
  double radius = sqrt( x*x + y*y);
  double theta = atan2(-y,x);
  double newTheta = theta + (theta0 *Deg2Rad);
  
  
  xNew = -radius * cos(newTheta) + X0;
  yNew = -radius * sin(newTheta) + Y0;
   
}


void Callback(const trimble_gps::GpsState& msg) {

  
  double x, y;
  double xNew, yNew;
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
  //ROS_INFO("Raw lat : %.8f  Raw lon: %.8f",psi, lambda);
  wgs2htz.ToHtz(psi, lambda, x, y);
  convertToLocalMap(x,y,xNew,yNew);
  
  
  
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.pose.pose.position.x = xNew ;
  pose_msg.pose.pose.position.y = yNew ;
  //ROS_INFO("NO offset x: %f  y: %f",xNew, yNew);
  pose_msg.pose.pose.position.z = 0.0 ;
  //make sure this is theta0 
  yaw = 180 + wgs2htz.GetLocalZeroHeading() -msg.Heading ;// /180.0 * M_PI;
  
  if(yaw > 180){
    yaw = -360 + yaw;
    
  }
  if(yaw<-180){
    yaw = +360 +yaw;
  }
 
  
  yaw = yaw*Deg2Rad;
  
  //move it to rear axes
  double antennaDisp = 0.34;
  xNew = xNew + antennaDisp*cos(yaw+M_PI);
  yNew = yNew + antennaDisp*sin(yaw+M_PI);
  pose_msg.pose.pose.position.x = xNew ;
  pose_msg.pose.pose.position.y = yNew ;
  
  //ROS_INFO("x: %.10f  y: %.10f",xNew, yNew);

  //ROS_INFO("Yaw : %f original %f ", yaw,  msg.Heading);
   
  pose_msg.pose.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
  pose_msg.header.frame_id = "world";
  pose_msg.header.stamp = ros::Time::now();
  //  pose_msg.header.stamp = msg.header.stamp;
  
  pose_pub.publish(pose_msg);  

   
}




int main(int argc, char ** argv)
{
  ros::init(argc, argv, "gps_converter");
  // conversion testing
  double x, y;
  //point2
  double x2,y2;
  double xNew,yNew;
  
  //new base COORDINATES
  //double psi = 37.41069084; // latitude in deg 3/13
  //double lambda = -122.02365480; // longitude in deg 3/13
  double psi = 37.41069079; // latitude in deg 3/27
  double lambda = -122.02365500; // longitude in deg 3/27
  
  //new base measure 1 FOR HEADING CALIBRATION
  //measure 2
  //double psiH = 37.4105547; //latitude 3/14
  //double lambdaH = -122.0236956; //longitude 3/14
  double psiH = 37.41012570; //latitude 3/27
  double lambdaH = -122.0238417; //longitude 3/27
    
    //ROS_INFO("psi: %f  lambda: %f",psi,lambda);
   
  const double psi0 = psi;
  const double lambda0 = lambda;
  const double psi2 = psiH;
  const double lambda2 = lambdaH;

  //set local base
  wgs2htz.SetBasePoint(psi0,lambda0);
  //set second point for zero heading
  wgs2htz.SetZeroHeading(psi2,lambda2);
  
  wgs2htz.ToHtz(psi0, lambda0, x, y);  
  ROS_INFO("Base x: %f  Base y: %f",x,y);
  convertToLocalMap(x,y,xNew,yNew);
  ROS_INFO("Map Reference Point1 x: %f Point1 y: %f",xNew,yNew);
  wgs2htz.ToHtz(psi2, lambda2, x2, y2);
  ROS_INFO("Base Point2 x: %f Point2 y: %f",x2,y2);   
  convertToLocalMap(x2,y2,xNew,yNew);
  ROS_INFO("Map Reference Point2 x: %f Point2 y: %f",xNew,yNew);
  ROS_INFO("Local Zero Heading: %f",wgs2htz.GetLocalZeroHeading());
      
  //just for testing
  if(0){
    //BASE
    wgs2htz.ToHtz(psi, lambda, x, y);  
    ROS_INFO("Base x: %f  Base y: %f",x,y);
    convertToLocalMap(x,y,xNew,yNew);
    ROS_INFO("Map Reference Point1 x: %f Point1 y: %f",xNew,yNew);
      
    
    
//     double psi2 = 37.4105547;//convert(3724.6004777231);//latitude
//     double lambda2 = -122.0236956;//-convert(12201.4327132784);//longitude
    double psi2 = 37.41001316; //latitude 3/13
    double lambda2 = -122.02387887;// longitude 3/13   
    
    //double psi2 = 37.4105604;//convert(3724.6004777231);//latitude
    //double lambda2 = -122.0236943;//-convert(12201.4327132784);//longitude
    //double psi2 = convert(3724.6004777231);//latitude
    //double lambda2 = -convert(12201.4327132784);//longitude
    
    wgs2htz.ToHtz(psi2, lambda2, x2, y2);
    ROS_INFO("Base Point2 x: %f Point1 y: %f",x2,y2);
    double angle = 180/M_PI * atan2(y-y2,x-x2);
    ROS_INFO("angle %f",angle);
    
    convertToLocalMap(x2,y2,xNew,yNew);
    //should be x 25.16 and y -13.07 
    ROS_INFO("Map Point2 x: %f Point1 y: %f",xNew,yNew);
    
    /*
    double psi2 = convert(3724.6004777231);//latitude
    double lambda2 = -convert(12201.4327132784);//longitude
    wgs2htz.ToHtz(psi2, lambda2, x, y);
    convertToLocalMap(x,y,xNew,yNew);
    ROS_INFO("Point2 x: %f Point1 y: %f",xNew,yNew);
    
    //point3
    double psi3 = convert(3724.59959132333);//latitude
    double lambda3 = -convert(12201.4276713874);//longitude
    wgs2htz.ToHtz(psi3, lambda3, x, y);
    convertToLocalMap(x,y,xNew,yNew);
    ROS_INFO("Point3 x: %f Point3 y: %f",xNew,yNew);
    
    //point4
    double psi4 = convert(3724.6279472851);//latitude
    double lambda4 = -convert(12201.418274683);//longitude
    wgs2htz.ToHtz(psi4, lambda4, x, y);
    convertToLocalMap(x,y,xNew,yNew);
    ROS_INFO("Point4 x: %f Point4 y: %f",xNew,yNew);
    
    //point5
    double psi5 = convert(3724.65341745761);//latitude
    double lambda5 = -convert(12201.411063967);//longitude
    wgs2htz.ToHtz(psi5, lambda5, x, y);
    convertToLocalMap(x,y,xNew,yNew);
    ROS_INFO("Point5 x: %f Point5 y: %f",xNew,yNew);
    
    //point6
    double psi6 = convert(3724.64933987638);//latitude
    double lambda6 = -convert(12201.4109072261);//longitude
    wgs2htz.ToHtz(psi6, lambda6, x, y);
    convertToLocalMap(x,y,xNew,yNew);
    ROS_INFO("Point6 x: %f Point6 y: %f",xNew,yNew);
    
    //point7
    double psi7 = convert(3724.64878060625);//latitude
    double lambda7 = -convert(12201.3891438243);//longitude
    wgs2htz.ToHtz(psi7, lambda7, x, y);
    convertToLocalMap(x,y,xNew,yNew);
    ROS_INFO("Point7 x: %f Point7 y: %f",xNew,yNew);
    
    //point8
    double psi8 = convert(3724.64611190194);//latitude
    double lambda8 = -convert(12201.3919845218);//longitude
    wgs2htz.ToHtz(psi8, lambda8, x, y);
    convertToLocalMap(x,y,xNew,yNew);
    ROS_INFO("Point8 x: %f Point8 y: %f",xNew,yNew);
    */
    
  }
  //end conversion testing
  
  ros::NodeHandle n;
  pose_pub= n.advertise<geometry_msgs::PoseWithCovarianceStamped>("Pose_gps", 100);

  //  ros::Subscriber sub = n.subscribe<nrc_msgs::GpsState>("trimble_gps", 100, boost::bind(Callback, _1, pose_pub));
  ros::Subscriber sub = n.subscribe("trimble_gps", 100, Callback);
  ros::spin();
      
  return 0;
}
