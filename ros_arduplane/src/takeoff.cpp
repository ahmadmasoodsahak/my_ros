#include <ros/ros.h>
#include <mavros_msgs/CommandTOL.h>


mavros_msgs::CommandTOL takeoff;


int main(int argc, char **argv){
    ros::init(argc, argv, "takeoff_node");
    ros::NodeHandle nh;

    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    takeoff.request.altitude = 3;


    if(takeoff_client.call(takeoff) == true){
        ROS_INFO("Takeoff yapildi");
    }
}