#include <ros/ros.h>
#include "flux_msgs/Velocity.h"

void callback(const flux_msgs::VelocityConstPtr& msg){
    ROS_INFO_STREAM(*msg);
}
int main(int argc,char **argv){
    ros::init(argc,argv,"some_node"); // node thor_local_planner
    ros::NodeHandle n;
    ROS_INFO("Thor local mimic planner");
    ros::Subscriber path_sub     = n.subscribe("/blah",10,callback);
    ros::spin();
}