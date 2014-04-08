#include <ros/ros.h>
#include "serial_comm.h"
#include <string.h>
#include "std_msgs/String.h"

using namespace std;

class RosToSerialBridge {
    private:
        serial_comm* comm_port;
    public:
        RosToSerialBridge(ros::NodeHandle nh) {
            string port;
            nh.param<string>("port", port, string(""));
            comm_port = new serial_comm();
            comm_port->Connect(port);

            ros::Publisher port_out_pub = nh.advertise<std_msgs::String>("/novatel_port_out", 1000);
            ros::Publisher port_in_pub = nh.advertise<std_msgs::String>("/novatel_port_in", 1000);
            ros::Subscriber port_sub = nh.subscribe("/novatel_port_in", 1000, &RosToSerialBridge::portWrite, this); 

            ros::Rate poll_rate(100); 

            while (ros::ok()) {
                std_msgs::String msg;

                string data = comm_port->safeRead();
                msg.data= data;
                if (data.length() > 0) {  
                    port_out_pub.publish(msg);
                }


                ros::spinOnce();
                poll_rate.sleep(); 
            }
            comm_port->Close();
            delete comm_port; 
        }

        void portWrite(const std_msgs::String::ConstPtr& msg) {
            string to_write = msg->data;
            cout << "received: " << to_write << endl;
            comm_port->safeWrite( to_write + "\r\n");
        }

        
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "novatel_node");
    ros::NodeHandle nh("~");

    RosToSerialBridge bridge(nh);


}
