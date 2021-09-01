#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


void fakeData(nav_msgs::Path& pth){
	pth.poses.clear();
	float X[45] = {10,	12	,12	,14	,16	,18	,18	,18	,18	,18	,18	,18	,18	,18	,18	,18	,20	,22	
                ,24	,26	,28	,30	,32	,34	,36	,38	,38	,38	,40	,42	,42	,42	,42	,42	,42	,42	,42	,
                44	,44	,46	,46	,48	,48	,48	,50};
   float Y[45] = {10,12,14	,16	,18	,20	,22,24	,26	,28	,30	,32	,34	,36	,38	,40	,42	,40	,38	
                ,36	,34	,32	,30	,28	,26	,24	,22	,20	,18	,20	,22	,24	,26	,28	,30	,32	,34	,36	
                ,38	,40	,42	,44	,46	,48	,50};
   // float Y[20];
   // for (int j = 0;j<20;j++)
   // {
   //     Y[j] = 25;
   //     //std::cout<<Y[j]<<std::endl;
   // }
   int N = sizeof(X)/sizeof(X[0]);
	for(int i = 0;i < N;i++){
		geometry_msgs::PoseStamped pt;
		pt.pose.position.x = X[i];
		pt.pose.position.y = Y[i];

		pth.poses.push_back(pt);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "FakePathPublisher");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1.0);

	ros::Publisher pub_fakePath = nh.advertise<nav_msgs::Path>("/Trajectory",10);
	
	nav_msgs::Path pth;
	while(ros::ok()){
		ros::spinOnce();
		fakeData(pth);
		pub_fakePath.publish(pth);

		loop_rate.sleep();
	}
}
