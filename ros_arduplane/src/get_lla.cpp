#include <new_functions.hpp>


int main(int argc, char** argv){
    ros::init(argc, argv, "get_lla_node");
	ros::NodeHandle new_node;

    //initialize control publisher/subscribers
	init_publisher_subscriber(new_node);

    ros::Rate rate(2.0);

    while (ros::ok()){
        get_all_data();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}