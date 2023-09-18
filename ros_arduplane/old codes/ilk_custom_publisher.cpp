#include <ros/ros.h>
#include <ros_arduplane/robot_msg.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "ilk_custom_msg");
    ros::NodeHandle nh;

    ros::Rate looprate(5);

    ros::Publisher pub = nh.advertise<ros_arduplane::robot_msg>("/ilk_custom_topic", 10);

    while (ros::ok)
    {
        ros_arduplane::robot_msg msg;
        
        msg.robot_status = "good";
        msg.distance = 10.5;

        pub.publish(msg);

        looprate.sleep();
    }
}