#include<iostream>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
using namespace std;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ext_speed_node");
	std_msgs::Float64 ext_speed;
	float speed;
	std::cout << "Specify speed :";
	std::cin >> speed;
	ext_speed.data = speed;
	ros::NodeHandle nh;
  	ros::Publisher ext_pub = nh.advertise<std_msgs::Float64>("/external_speed", 50);
  	while(ros::ok()){
  		ext_pub.publish(ext_speed);
    	ros::spinOnce(); 
    }
}