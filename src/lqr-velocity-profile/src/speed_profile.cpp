//Generic libraries.
#include "math.h"
#include "tf/tf.h"
#include "iostream"
#include "vector"
//ROS libraries.
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include <visualization_msgs/Marker.h>
// #include "flux_msgs/Velocity.h"
// #include "flux_msgs/VelocityArray.h"
// #include <dynamic_reconfigure/server.h>
// #include <dynamic_parameters/paramsConfig.h>

int m;
std::vector<double> X,Y,odomX,odomY;
std::vector<double> speed_array;

std_msgs::Float64 vel_msg;
ros::Publisher vel_pub;
void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
	//std::cerr <<"\n PathCallback" ;
	int m = msg->poses.size();

	for(int i = 0;i<m;i++)
	{
		X.push_back(msg->poses[i].pose.position.x);
		Y.push_back(msg->poses[i].pose.position.y);

	}

}
void speedCallback(const std_msgs::Float64::ConstPtr& msg)
{
	//std::cerr << "\n SpeedCallback" ;
	//std::cout << msg->data ;
	speed_array.push_back(msg->data);
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//std::cerr << "\n OdomCallback" ;
	double x_location = msg->pose.pose.position.x;
	double y_location = msg->pose.pose.position.y;
}
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "speed_node");
	ros::NodeHandle n;
	ros::Subscriber traj_sub = n.subscribe("/spline",1,pathCallback);
	ros::Subscriber vel_sub = n.subscribe("/velocity",1,speedCallback);
	ros::Subscriber odom_sub = n.subscribe("/absolute_pose",1,odomCallback);


	vel_pub = n.advertise<std_msgs::Float64> ("/speed",1,true);

	for(int i=0;i<m;i++)
	{
		std::cout<< speed_array[i];
	}
	
	while(ros::ok())
	{
		ros::Rate(15).sleep();

		ros::spin();


	}


	



};

