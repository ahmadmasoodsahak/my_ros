#include <ros/ros.h>
#include <mavros_msgs/ParamSet.h>

mavros_msgs::ParamSet param;


int main(int argc, char **argv){
    ros::init(argc, argv, "param_node");
    ros::NodeHandle nh;

    ros::ServiceClient param_client = nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");

    param.request.param_id="RC4_MAX";
    param.request.value.real=1900.0;


    if(param_client.call(param) == true){
        ROS_INFO("parametre ayarlandi!");
    }
}