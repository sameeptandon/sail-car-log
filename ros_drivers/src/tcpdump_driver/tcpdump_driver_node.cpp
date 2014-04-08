#include <ros/ros.h>
#include <string.h>
#include <signal.h>

using namespace std;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_tcpdump_node");
    ros::NodeHandle nh("~");
    int lidar_pid = 0; 
    string outputPcap;
    nh.param<string>("outputPcap", outputPcap, string(""));

    int pid = fork();
    if (pid == 0) {

        char* const parmList[] = {(char *)"/usr/sbin/tcpdump",
            (char*)"-i" ,
            (char*)"eth0",
            (char*)"-w",
            (char*)strdup(outputPcap.c_str()),
            (char*)"udp port 2368 or udp port 8308", 
            NULL}; 

        int rc = execv("/usr/sbin/tcpdump", parmList);
        if (rc == -1) {
            cout << "error loading lidar script" << endl;
            return 0; 
        }
    } else { 
        lidar_pid = pid;
    }

    ros::spin(); // wait for exit signal 

    
    kill(lidar_pid, SIGTERM); 

}
