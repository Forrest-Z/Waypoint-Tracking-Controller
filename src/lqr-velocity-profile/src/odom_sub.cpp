#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "nav_msgs/Path.h"
#include <iostream>
void callbackGetTrajectory(const nav_msgs::Path::ConstPtr& msg)
{
  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/absolute_pose", 50);
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  tf::TransformBroadcaster odom_broadcaster;
  std ::vector<double>X(msg->poses.size()),Y(msg->poses.size()),orientationX(msg->poses.size()); 
   //compute odometry in a typical way given the velocities of the robot
  //Forming arrays for X,Y co-ordinates from pose data
  //std::cout << "Path Size : " << msg->poses.size();
  for (int i = 0;i<X.size();i++)
  {
    //std::cerr<< i << std::endl;
    //std::cerr<<msg->poses[i].pose.position.y << std::endl;
    X[i] = msg->poses[i].pose.position.x;
    Y[i] = msg->poses[i].pose.position.y;
    float x1 = X[i]; float x2 = X[i+1];
    float y1 = Y[i]; float y2 = Y[i+1];
        //Orinetation w.r.t X-axis
        //Positive angles from 0-180 deg CCW (1st,2nd quadrants), Negative angles from 0-179 deg CW (4th, 3rd quadrants)
    orientationX[i] = atan2((y2-y1),(x2-x1)); 
  }
  for(int i=0;i<X.size();i++)
  {
    double x = x+ X[i] ;
    double y = y+Y[i];
    double th = th+orientationX[i];

    // double vx = 0.1;
    // double vy = -0.1;
    // double vth = 0.1;
    // double dt = (current_time - last_time).toSec();
    // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    // double delta_th = vth * dt;

    // x += 0;
    // y += 0;
    // th +=0;

      //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

      //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      // odom.child_frame_id = "base_link";
      // //set the velocity
      // odom.child_frame_id = "base_link";
      // odom.twist.twist.linear.x = vx;
      // odom.twist.twist.linear.y = vy;
      // odom.twist.twist.angular.z = vth;

      //publish the message
      odom_pub.publish(odom);
   
    }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  boost::shared_ptr<nav_msgs::Path const> path;
  ros::NodeHandle n;
  ros::Subscriber sub_path = n.subscribe("/ThorPlanner/GlobalPath",1,&callbackGetTrajectory);
  ros::Time current_time, last_time;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/absolute_pose", 50);
  ros::Rate r(1.0);
  while(ros::ok()){

    ros::spinOnce();

    current_time = ros::Time::now();

   
    last_time = current_time;
    r.sleep();
  }
}
