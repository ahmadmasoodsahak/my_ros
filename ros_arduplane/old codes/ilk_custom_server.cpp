#include <ros/ros.h>
#include <ros_arduplane/robot_srv.h>

bool server_func(ros_arduplane::robot_srv::Request &req, ros_arduplane::robot_srv::Response &res){

    res.toplam = req.a + req.b;

    ROS_INFO("%ld + %ld = %ld", req.a, req.b, res.toplam);

    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ilk_custom_srv");
    ros::NodeHandle nh;


    ros::ServiceServer server = nh.advertiseService("/ilk_custom_server",server_func);

    ros::spin();
}