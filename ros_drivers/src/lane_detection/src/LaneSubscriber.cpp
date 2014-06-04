#include "ros/ros.h"
#include "lane_detection/LaneOutput.h"

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

}


int main(int argc, char **argv){
    ros::init(argc,argv,"LaneSubscriber");
    ros::NodeHandle nh_;
    ros::Subscriber sub = nh_.subscribe("/lane_detection/LaneOutput",1,laneCallback);
    ros::spin();
    return 0;
}
