#include <ros/ros.h>
#include <turtlesim/TeleportAbsolute.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "ilk_service_client");
    ros::NodeHandle nh;

    ros::Rate looprate(20);

    ros::ServiceClient client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

    turtlesim::TeleportAbsolute tel;

    tel.request.x = 5.0;
    tel.request.y = 4.0;

    if(client.call(tel)){
        ROS_INFO("service calisti.");
    }
    else{
        ROS_INFO("service calismadi.");
    }
    

}