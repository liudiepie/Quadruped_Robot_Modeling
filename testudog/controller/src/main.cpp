#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <iostream>
#include "../include/testudog/testudog.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "gazebo_controller");
    ros::NodeHandle nh;
	std::array<std::string,12> command_topic = {
		"/testudog_controller/front_left_rolling_controller/command",
		"/testudog_controller/front_left_pitching_controller/command",
		"/testudog_controller/front_left_knee_controller/command",
		"/testudog_controller/front_right_rolling_controller/command",
		"/testudog_controller/front_right_pitching_controller/command",
		"/testudog_controller/front_right_knee_controller/command",
		"/testudog_controller/back_left_rolling_controller/command",
		"/testudog_controller/back_left_pitching_controller/command",
		"/testudog_controller/back_left_knee_controller/command",
		"/testudog_controller/back_right_rolling_controller/command",
		"/testudog_controller/back_right_pitching_controller/command",
		"/testudog_controller/back_right_knee_controller/command"
	};
	std::array<ros::Publisher,12> pub;
	for (int i = 0; i <= 11; i++){
		pub.at(i) = nh.advertise<std_msgs::Float64>(command_topic.at(i), 1);
	}

	robot::Testudog test;
	ros::Subscriber sub = nh.subscribe("testudog_imu/body_orientation", 1, &robot::Testudog::get_imu, &test);

	std::array<std_msgs::Float64,12> command_msg;
	ros::Rate loop_rate(60);

	// std_msgs::String msg;
	// std::stringstream ss;

    while(ros::ok()){
		test.circle();
		int k = 0;
		for (int j = 0; j <= 3; j++){
			for (int i = 0; i <= 2; i++){
				command_msg.at(k).data = test.get_joint_angle()(i,j);
				pub.at(k).publish(command_msg.at(k));


				ROS_INFO("%f", test.get_joint_angle()(i,j));
				
				k = k + 1;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}




