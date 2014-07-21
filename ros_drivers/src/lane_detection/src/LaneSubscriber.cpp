#include "ros/ros.h"
#include "lane_detection/LaneOutput.h"

#include <canlib.h>
#include "RXTX_CANCom.h" // Class for Kvaser Can Handling
#include "RXTX_ConvData.h" // 

//----------------Define---------------------------------------
CCANCom CANCom;


void laneCallback(const lane_detection::LaneOutput::ConstPtr& msg){
    float leftLane[3][10];//[x,y,z][point 1, point 2, ... , point 10]
    float rightLane[3][10];
    float centerLine[3][10];
    
    for(int i=0;i<10;i++){
        for(int j=0;j<3;j++){
            leftLane[j][i] = msg->left[i*3+j];
            rightLane[j][i] = msg->right[i*3+j];
            centerLine[j][i] = (leftLane[j][i]+rightLane[j][i])/2.0;
        }
    }

    // CAN transimition
    bool FLG = false;
    CAN_DATA_FORM CAN;
    int i;
    
    int ID = 0x730;
    for( i=0;i<8;i++ ){CAN.bData[i] = 0;}
    CAN.ID_730.yr1 = (uint) ( ( centerLine[0][0] + 327.68 ) /0.01 ) ; 
    CAN.ID_730.yr2 = (uint) ( ( centerLine[0][1] + 327.68 ) /0.01 ) ; 
    CAN.ID_730.yr3 = (uint) ( ( centerLine[0][2] + 327.68 ) /0.01 ) ; 
    CAN.ID_730.yr4 = (uint) ( ( centerLine[0][3] + 327.68 ) /0.01 ) ;
    CANCom.transmit(ID,8,CAN.bData);

    ID = 0x731;
    for( i=0;i<8;i++ ){CAN.bData[i] = 0;}
    CAN.ID_731.yr5 = (uint) ( ( centerLine[0][4] + 327.68 ) /0.01 ) ; 
    CAN.ID_731.yr6 = (uint) ( ( centerLine[0][5] + 327.68 ) /0.01 ) ; 
    CAN.ID_731.yr7 = (uint) ( ( centerLine[0][6] + 327.68 ) /0.01 ) ; 
    CAN.ID_731.yr8 = (uint) ( ( centerLine[0][7] + 327.68 ) /0.01 ) ; 
    CANCom.transmit(ID,8,CAN.bData);

    ID = 0x732;
    for( i=0;i<8;i++ ){CAN.bData[i] = 0;}
    CAN.ID_732.yr9 = (uint) ( ( centerLine[0][8] + 327.68 ) /0.01 ) ; 
    CAN.ID_732.yr10 = (uint) ( ( centerLine[0][9] + 327.68 ) /0.01 ) ; 
    CANCom.transmit(ID,8,CAN.bData);

    ID = 0x733;
    for( i=0;i<8;i++ ){CAN.bData[i] = 0;}
    CAN.ID_733.xr1 = (uint) ( ( centerLine[2][0] ) /0.01 ) ; 
    CAN.ID_733.xr2 = (uint) ( ( centerLine[2][1] ) /0.01 ) ; 
    CAN.ID_733.xr3 = (uint) ( ( centerLine[2][2] ) /0.01 ) ; 
    CAN.ID_733.xr4 = (uint) ( ( centerLine[2][3] ) /0.01 ) ; 
    CANCom.transmit(ID,8,CAN.bData);

    ID = 0x734;
    for( i=0;i<8;i++ ){CAN.bData[i] = 0;}
    CAN.ID_734.xr5 = (uint) ( ( centerLine[2][4] ) /0.01 ) ;
    CAN.ID_734.xr6 = (uint) ( ( centerLine[2][5] ) /0.01 ) ;
    CAN.ID_734.xr7 = (uint) ( ( centerLine[2][6] ) /0.01 ) ;
    CAN.ID_734.xr8 = (uint) ( ( centerLine[2][7] ) /0.01 ) ;   
    CANCom.transmit(ID,8,CAN.bData);


    ID = 0x735;
    for( i=0;i<8;i++ ){CAN.bData[i] = 0;}
    CAN.ID_735.xr9 = (uint) ( ( centerLine[2][8] ) /0.01 ) ; 
    CAN.ID_735.xr10 = (uint) ( ( centerLine[2][9] ) /0.01 ) ; 
    FLG = CANCom.transmit(ID,8,CAN.bData);
    
    //if(FLG){ ROS_DEBUG("Send Pose[x, y, heading]: %f, %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y, yaw ); }
    //else { ROS_ERROR("CAN Send Err"); }
    
}


int main(int argc, char *argv[]){
  
    ros::init(argc,argv,"LaneSubscriber");
    ros::NodeHandle nh_;
  // -----------------Build CAN communication -----------------
  //--Define--
  int channel = 0;

  ROS_INFO("start");

  //-Open can driver-
  errno = 0;

  if (argc != 2 || (channel = atoi(argv[1]), errno) != 0) {
    ROS_DEBUG("usage %s channel\n", argv[0]);
    exit(1);
  }
  else {
    ROS_DEBUG("Writing messages on channel %d\n", channel);
  }

  /* Open channels, parameters and go on bus */
  CANCom.connect(channel);

  if ( ! CANCom.isConnected() ) {
    ROS_ERROR("CAN Open Channel %d Failed\n", channel);
    exit(1);
  }
  else {
    ROS_INFO("CAN Open Channel %d Succeeded\n", channel);
  }
  //--------------------------------------------------------
    ros::Subscriber sub = nh_.subscribe("/lane_detection/LaneOutput",1,laneCallback);
    ros::spin();
    return 0;
}
