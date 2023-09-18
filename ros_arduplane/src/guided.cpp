#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>    //  arm vermek
#include <geometry_msgs/PoseStamped.h>  //  local point vermek
#include <mavros_msgs/SetMode.h>        //  mode ayarlamak
#include <nav_msgs/Odometry.h>          //  local konum bilgi almak
#include <mavros_msgs/State.h>          //  IHA durumunu anlamak


mavros_msgs::CommandBool com;
geometry_msgs::PoseStamped pose;
mavros_msgs::SetMode mode;
nav_msgs::Odometry odom;
mavros_msgs::State state;

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg){
    odom=*msg;
}

void state_cb(const mavros_msgs::State::ConstPtr &msg){
    state=*msg;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "guided_node");
    ros::NodeHandle nh;

    ros::ServiceClient com_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/et_mode");
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, odom_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    ros::Rate looprate(10);

    while(ros::ok() && !state.connected){
        ros::spinOnce();
        looprate.sleep();
    }

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    for(int i=100; ros::ok() && i>0; --i){
        pose_pub.publish(pose);
        ros::spinOnce();
        looprate.sleep();
    }

    mode.request.custom_mode="AUTO";

    if(mode_client.call(mode) == true){
        if(mode.response.mode_sent == true){
            ROS_INFO("Mode GUIDED");
        }

        com.request.value = true;

        if(com_client.call(com) == true){
            if(com.response.result == 0){
                ROS_INFO("ARMED!!!");
            }
        }

        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 5.0;

        while (ros::ok()){
            ROS_INFO("x:%f, y:%f, z:%f, mode:%s", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, state.mode.c_str());
            pose_pub.publish(pose);
            ros::spinOnce();
            looprate.sleep();
        }   
    }
}
