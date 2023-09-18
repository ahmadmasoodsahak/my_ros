#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/BatteryState.h>
#include "mavros_msgs/VFR_HUD.h"
#include <cmath>
#include <ctime>
#include <jsoncpp/json/json.h>
#include <nlohmann/json.hpp>
#include <map>




mavros_msgs::State current_state;
sensor_msgs::NavSatFix current_gps;
nav_msgs::Odometry current_alt;
geometry_msgs::PoseStamped current_pose;
sensor_msgs::BatteryState current_bat;
mavros_msgs::VFR_HUD current_speed;


int teamNo = 74;
float lat;
float lng;
int alt;
int pitch_degrees;
int yaw_degrees;
int roll_degrees;
int speed;
int bat;
int automatic;
int locking = 0;
int target_center_X = 0;
int target_center_Y = 0;
int target_width = 0;
int target_height = 0;
int hour;
int minute;
int second;
int millisecond;


ros::Subscriber state_sub;
ros::Subscriber gps_sub;
ros::Subscriber alt_sub;
ros::Subscriber pose_sub;
ros::Subscriber bat_sub;
ros::Subscriber speed_sub;


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;

	if (current_state.mode == "AUTO" || current_state.mode == "GUIDED" || current_state.mode == "LOITER" ||
        current_state.mode == "CIRCLE" || current_state.mode == "RTL" || current_state.mode == "LAND") {
        automatic = 1;
    } else {
        automatic = 0;
    }
}

void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){

    current_gps = *msg;

	lat = current_gps.latitude;
	lng = current_gps.longitude;

	// Extract the Unix timestamp
    ros::Time gps_time = current_gps.header.stamp;
    time_t unix_time = gps_time.sec;

    // Convert to a struct tm (timeinfo)
    struct tm* timeinfo;
    timeinfo = std::gmtime(&unix_time);

    // Extract hour, minute, second, and millisecond
    hour = timeinfo->tm_hour;
    minute = timeinfo->tm_min;
	second = timeinfo->tm_sec;
    millisecond = gps_time.nsec / 1e6;
}

void alt_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  	current_alt = *msg;

	alt = current_alt.pose.pose.position.z;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;

	float linearposx=current_pose.pose.position.x;
   	float linearposy=current_pose.pose.position.y;
   	double quatx= current_pose.pose.orientation.x;
   	double quaty= current_pose.pose.orientation.y;
	double quatz= current_pose.pose.orientation.z;
   	double quatw= current_pose.pose.orientation.w;

  	tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

	if (yaw < 0) {
        yaw += 2 * M_PI;
    }
	if (roll > M_PI) {
        roll -= 2 * M_PI;
    }
    if (pitch > M_PI) {
        pitch -= 2 * M_PI;
    }

    pitch_degrees = -(static_cast<int>(pitch * 180.0 / M_PI));
	yaw_degrees = static_cast<int>(yaw * 180.0 / M_PI);
	roll_degrees = static_cast<int>(roll * 180.0 / M_PI);
}

void speed_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg) {

	current_speed = *msg;
    speed = static_cast<int>(current_speed.airspeed);
}

void bat_cb(const sensor_msgs::BatteryState::ConstPtr& msg) {
	current_bat = *msg;
	bat = static_cast<int>(current_bat.percentage * 100);
}

void get_all_data() {
  // Verileri JSON formatında bir string olarak döndür.
  	std::string json_data = "{\n";
  	json_data += "  \"takim_numarasi\": " + std::to_string(teamNo) + ",\n";
  	json_data += "  \"iha_enlem\": " + std::to_string(lat) + ",\n";
  	json_data += "  \"iha_boylam\": " + std::to_string(lng) + ",\n";
  	json_data += "  \"iha_irtifa\": " + std::to_string(alt) + ",\n";
  	json_data += "  \"iha_dikilme\": " + std::to_string(pitch_degrees) + ",\n";
  	json_data += "  \"iha_yonelme\": " + std::to_string(yaw_degrees) + ",\n";
  	json_data += "  \"iha_yatis\": " + std::to_string(roll_degrees) + ",\n";
  	json_data += "  \"iha_hiz\": " + std::to_string(speed) + ",\n";
  	json_data += "  \"iha_batarya\": " + std::to_string(bat) + ",\n";
  	json_data += "  \"iha_otonom\": " + std::to_string(automatic) + ",\n";
  	json_data += "  \"iha_kilitlenme\": " + std::to_string(locking) + ",\n";
  	json_data += "  \"hedef_merkez_X\": " + std::to_string(target_center_X) + ",\n";
  	json_data += "  \"hedef_merkez_Y\": " + std::to_string(target_center_Y) + ",\n";
  	json_data += "  \"hedef_genislik\": " + std::to_string(target_width) + ",\n";
  	json_data += "  \"hedef_yukseklik\": " + std::to_string(target_height) + ",\n";
  	json_data += "  \"gps_saati\": {\n";
  	json_data += "\t\"saat\": " + std::to_string(hour) + ",\n";
  	json_data += "\t\"dakika\": " + std::to_string(minute) + ",\n";
  	json_data += "\t\"saniye\": " + std::to_string(second) + ",\n";
  	json_data += "\t\"milisaniye\": " + std::to_string(millisecond) + "\n";
  	json_data += "  }\n";
  	json_data += "}";

  	// Verileri ROS_INFO() kullanarak ekrana yazdır.
  	ROS_INFO("%s", json_data.c_str());
}


int init_publisher_subscriber(ros::NodeHandle controlnode)
{
	std::string ros_namespace;
	if (!controlnode.hasParam("namespace"))
	{

		ROS_INFO("using default namespace");
	}else{
		controlnode.getParam("namespace", ros_namespace);
		ROS_INFO("using namespace %s", ros_namespace.c_str());
	}
	state_sub = controlnode.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    gps_sub = controlnode.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/raw/fix", 10, gps_cb);
	alt_sub = controlnode.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, alt_cb);
	pose_sub = controlnode.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
	bat_sub = controlnode.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 10, bat_cb);
	speed_sub = controlnode.subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud", 10, speed_cb);
	return 0;
}