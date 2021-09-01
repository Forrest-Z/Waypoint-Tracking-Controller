//Generic libraries.
#include "math.h"
#include "tf/tf.h"
#include "iostream"
#include "vector"
//ROS libraries.
#include "ros/ros.h"
#include "std_msgs/Bool.h"
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


#define  PI 3.1416
#define  n_max	500			//Max. spline sections.
#define  N_RET_VEL_MAX 20		//MAX N_RET_VEL
int path_size;
float min_dist;
int N_max;		
double rc_max;
double rc_min;
double v_max;
int n_ret_vel;
double sample_time;
double q11,q22,r11; 
std::vector <double> arrayRc;
std::vector <double> arrayVel;
double x_fin, y_fin,x_init,y_init;
double coefs[n_max][8];
double x_fin_prev=0, y_fin_prev=0, u=0.5, u_ant = 0.5, pose_d[3];
double actualPose[3];
double external_speed;
double v=0, ro=0, rc = 0;
double dist_location=1.2, dist_wheels=1.462;    
double max_steer_angle= 1.0472;  
double A[4],B[2],Q[4],R,Kr[2];
//Delay compensation variables
double buffer_vel[2][N_RET_VEL_MAX];
double evol_vel[2][N_RET_VEL_MAX]; // row 1 v, row 2 ro
int n_section = 0, section_change=0, n_sections = 0;
int init=0, init_odom=0;
int m;
double x[500],y[500];
bool stop=true,manual=true,obstacle=true;
std::vector<double> vel_array,X,Y;
ros::Publisher spline_pub;
ros::Publisher cmd_vel_pub;
ros::Publisher spline_points_pub;
ros::Publisher reference_pose_pub;
ros::Publisher predicted_pose_pub;
ros::Publisher speed_pub;
ros::Publisher steer_pub;
ros::Publisher de_pub;
ros::Publisher oe_pub;

nav_msgs::Odometry actualOdom;
std_msgs::Float64 external_speed_msg;

geometry_msgs::PoseStamped reference_pose;
geometry_msgs::PoseStamped pose_future;
geometry_msgs::Twist cmd_vel_msg;
//flux_msgs::Velocity speed;
//geometry_msgs::Twist cmd_vel_msg;
std_msgs::Float64 speed_msg;
std_msgs::Float64 steer_msg;
std_msgs::Float64 de_msg;
std_msgs::Float64 oe_msg;


void VehicleAtRest(double v);
void n_spline(double *x, double *y, double *o, double nu, int m);
void calculate_consign(double actualPose[3],double coefs[n_max][8],int n_section); 
float sol_equation(float coef[6], float x);
double Limitang(double ang);
void Dlqr(double K[2], double A[4], double B[2], double Q[4], double R);
void Multiply(double *origin1, double *origin2, long x, long y, long z, double *goal);
void mult_esc_mat(double esc, double *mat, int n, double *goal);
void sum_ident(double *mat, int n);
void Inverse2(double *origin, double *goal);
void Transposed2(double *origin, double *goal);
void Add(double *origin1, double *origin2, int nf, int nc, double *goal);
void Consigna_ro(double de, double oe, double Kr[2], double *ro);
void velGeneration ();

//**************************Dynamic Reconfigure************************************************//
// void callback(dynamic_parameters::paramsConfig &config, uint32_t level) {
//   ROS_INFO("Reconfigured");// Request: %d %f %s %s %d", 
//             // config.int_param, config.double_param, 
//             // config.str_param.c_str(), 
//             // config.bool_param?"True":"False", 
//             // config.size);
//   std::cerr << "Updated Parameters";
//   ROS_INFO("\nPARAMETERS:");
//   ROS_INFO("\tN_max: %d",N_max);
//   ROS_INFO("\tmin_dist: %lf",min_dist);
//   ROS_INFO("\trc_max: %lf",rc_max);
//   ROS_INFO("\trc_min: %lf",rc_min);
//   ROS_INFO("\tv_max: %lf",v_max);
//   ROS_INFO("\tn_ret_vel: %d",n_ret_vel);
//   ROS_INFO("\tsample_time: %lf",sample_time);
//   ROS_INFO("\tq11: %lf",q11);
//   ROS_INFO("\tq22: %lf",q22);
//   ROS_INFO("\tr11: %lf",r11);
// }

//**************************Spline publisher************************************************//
void trajectoryCallback(const boost::shared_ptr<nav_msgs::Path const>& msg) 
{
	// std::cerr << msg ;
	//std::cerr <<"Entered trajectory callback ..";
	 m=msg->poses.size();
	 for(int i = 0;i<m;i++)
	{
		X.push_back(msg->poses[i].pose.position.x);
		Y.push_back(msg->poses[i].pose.position.y);

	}
	 int j=1; 
	 //path_size = m;
     int n_points=0;
	 double dist;
  	 double o[2];  //m will be the maximum number of points if we took them all.
	 double u_interpol[10] = {0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9}; //We will get 10 points per section
	 //double u_interpol[20] = {0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,
	 				//1.2,1.3,1.4,1.5,1.6,1.7};//1.8,1.9};  //We will get 20 points per section
	 //double u_interpol[30] = {0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1
	 						//1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2.0,2.1,2.2,2.3,2.4,
							//2.5,2.6,2.7,2.8,2.9};  //We will get 30 points per section
	 nav_msgs::Path spline_path;
	 geometry_msgs::PoseStamped position;
	 geometry_msgs::Point p;
	 visualization_msgs::Marker points_spline;
	 points_spline.type = visualization_msgs::Marker::SPHERE_LIST;
	 points_spline.action = visualization_msgs::Marker::ADD;
	 points_spline.color.r=1.0;
	 points_spline.color.a=1.0;
	 points_spline.scale.x=1;
	 points_spline.scale.y=0.1;
	 points_spline.scale.z=0.1;

         //ROS_INFO("Path callback with init_odom=%d", init_odom);
    		
	if (init_odom==0) return;

        //If the trajectory has not changed, we exit so as not to recalculate it
	 x_fin = msg->poses[m-1].pose.position.x;
	 y_fin = msg->poses[m-1].pose.position.y;
	
	 
	 if ((fabs(x_fin - x_fin_prev)<0.5) && (fabs(y_fin - y_fin_prev)<0.5))
		return ;
	
	
	stop=false;
	
	x_fin_prev = x_fin;
	y_fin_prev = y_fin;

	n_section = 0;
	section_change = 0;
	n_sections = 0;
	u=0.5; u_ant=0.5;
	points_spline.points.clear();

	//New trajectory defined	 
	 	
	
	//The actual position of the vehicle is the initial point of the trajectory
	

	x[0]=actualPose[0];
	y[0]=actualPose[1];
	n_points++;
	p.x = x[0];
  	p.y = y[0];
	p.z = 0;
	points_spline.points.push_back(p);    

	
	for (int i=1; i<m; i++)  
	  {
                dist=sqrt((msg->poses[i].pose.position.x-x[j-1])*(msg->poses[i].pose.position.x-x[j-1])
					+(msg->poses[i].pose.position.y-y[j-1])*(msg->poses[i].pose.position.y-y[j-1]));
				//std::cerr<< dist;
		if (dist>min_dist)
		{
			x[j]=msg->poses[i].pose.position.x;
		    y[j]=msg->poses[i].pose.position.y;
		    p.x = x[j];
			p.y = y[j];
			p.z = 0;
			points_spline.points.push_back(p);
			//j++;
			n_points++;
			n_sections++;
		}
	  }

          o[0]=Limitang(actualPose[2]);
	  o[1]=atan2(y[n_points-1]-y[n_points-2],x[n_points-1]-x[n_points-2]);

          n_spline(x,y,o,0.00001,n_points);

          for (int i=0; i<n_sections; i++)
	  {
		for (int k=0; k<10; k++)
		{
			position.pose.position.x = coefs[i][0]+coefs[i][1]*u_interpol[k]+coefs[i][2]*u_interpol[k]*u_interpol[k]+coefs[i][3]*u_interpol[k]*u_interpol[k]*u_interpol[k];
			position.pose.position.y = coefs[i][4]+coefs[i][5]*u_interpol[k]+coefs[i][6]*u_interpol[k]*u_interpol[k]+coefs[i][7]*u_interpol[k]*u_interpol[k]*u_interpol[k];
                        position.pose.position.z = 0;
			spline_path.poses.push_back(position);
		}
	  }
	 //std::cerr << "***---***" << n_sections;
	 velGeneration();
	 n_section=0;
	 
	 spline_path.header.frame_id="map";
	 points_spline.header.frame_id = "map";

	 spline_pub.publish(spline_path);
	 spline_points_pub.publish(points_spline);
    	init=1;
		//std::cerr << "Completed";
	
}

//********************************Linear velocity profile generation***********************//

void velGeneration ()
{
	double der1_x, der1_y, der2_x, der2_y, der1, der2, aux;	

	for(n_section=0; n_section<=n_sections; n_section++)
	{	
		double rc_sum=0;
		double num_it=0;
		for(u=0; u<=1; num_it++)
		{	
			der1_x=(3*coefs[n_section][3]*u*u)+(2*coefs[n_section][2]*u)+coefs[n_section][1];
		    	der1_y=(3*coefs[n_section][7]*u*u)+(2*coefs[n_section][6]*u)+coefs[n_section][5];
		    	der1=der1_y/der1_x;
		    	der2_x=(6*coefs[n_section][3]*u)+(2*coefs[n_section][2]);
		    	der2_y=(6*coefs[n_section][7]*u)+(2*coefs[n_section][6]);
		    	der2=((der2_y*der1_x)-(der1_y*der2_x))/(der1_x*der1_x*der1_x);
		    	der2=fabs(der2);
		    	
			if (der2<0.001)
				rc=rc_max;
		   	else
			{
				aux=(1+der1*der1);
				rc=sqrt(aux*aux*aux);
				rc=rc/der2;
			}
			if (rc<rc_min)
				rc=rc_min;
		    	if (rc>rc_max)
				rc=rc_max;
			u=u+0.1; 
			rc_sum=rc_sum+rc;
			
		}
		rc_sum=rc_sum/num_it;
		if(arrayRc.size()<n_sections)
		arrayRc.push_back(rc_sum);
	}
	for(n_section=0; n_section<=n_sections; n_section++)
	{	double vel;
		if((n_sections-n_section)>5)
		vel=0.1*(v_max*arrayRc[n_section-2])/(rc_max)+0.2*(v_max*arrayRc[n_section-1])/(rc_max)+0.4*(v_max*arrayRc[n_section])/(rc_max)+0.2*(v_max*arrayRc[n_section+1])/(rc_max)+0.1*(v_max*arrayRc[n_section+2])/(rc_max);
		else 
		vel=(v_max*arrayRc[n_section])/(rc_max);

		if(arrayVel.size()<n_sections)
		arrayVel.push_back(vel);
		
	}

	
}

//********************************Odometry callback******************************************//
void odometryCallback(const boost::shared_ptr<nav_msgs::Odometry>& msg)
{
	double x_location, y_location, theta_location;

	actualOdom = *msg;
	x_location = actualOdom.pose.pose.position.x;
	y_location = actualOdom.pose.pose.position.y;
	theta_location = tf::getYaw(actualOdom.pose.pose.orientation);

	actualPose[0] = x_location + dist_location*cos(theta_location);
	actualPose[1] = y_location + dist_location*sin(theta_location);
	actualPose[2] = theta_location;
	//std::cout << "\n X:"<< fabs(x_location - actualPose[0])<< "\n Y: " << fabs(y_location- actualPose[1]);
	if ((fabs(x_fin - actualPose[0])<0.1) && (fabs(y_fin- actualPose[1])<0.1))
		stop=true;

	// if ((fabs(x_location - )<2.0) && (fabs(y_location- actualPose[1])<2.0))
	// 	stop=false;
	// if ((fabs(x_location - actualPose[0])<0) && (fabs(y_location- actualPose[1])<0))
	// 	stop=true;
	init_odom=1;

}

//********************************External Speed callback******************************************//

// void externalSpeedCallback(const boost::shared_ptr<std_msgs::Float64 const>& msg)
// {

// 	external_speed_msg = *msg;
// 	external_speed=external_speed_msg.data;
// }

void VehicleAtRest(double v)
{
	std::cout << "\n Entered callback";
	int rest_count = 0;
	if(v == 0)
		rest_count ++ ;
	if(rest_count > 3)
		std::cout << " \n Vehicle is Stationary.. " ; 
	

}
//********************************Manual Flag callback******************************************//

void manualCallback(const boost::shared_ptr<std_msgs::Bool const>& msg)
{
	if(msg->data == true)
		manual = true;
	if(msg->data == false)
		manual = false;
}

//********************************Obstacle Flag callback******************************************//

void obstacleCallback(const boost::shared_ptr<std_msgs::Bool const>& msg)
{
	if(msg->data == true)
		obstacle = true;
	if(msg->data == false)
		obstacle = false;
		
}
//**********************************ROS Timer*******************************************//

void timerCallback(const ros::TimerEvent& e) 
{
	if(manual==false && obstacle==false)
	{
	//std::cerr << "Entered callback..." << std::endl;
	double de=0, oe=0, v=0;
	//std::cerr << "declaration..." << std::endl;
	double futurePose[3];
	//std::cerr << "declaration OF Future..." << std::endl;
        if (init==0){
			//std::cerr << "init is zero..." << std::endl;
			return;
		}
	//Actual location propagating
	//std::cerr << "actual location propagating";
	futurePose[0] = actualPose[0];
	futurePose[1] = actualPose[1];
	futurePose[2] = actualPose[2];
	//std::cerr << "actual location done";
	for (int i=0; i<n_ret_vel;i++)
	{
		//std::cerr << "actual propagating";
		futurePose[0] = futurePose[0] + evol_vel[0][i]*cos(futurePose[2])*sample_time;
		futurePose[1] = futurePose[1]+ evol_vel[0][i]*sin(futurePose[2])*sample_time;
		futurePose[2] = futurePose[2]+ evol_vel[1][i]*sample_time;
	      }
		//std::cerr << "consign function";
	calculate_consign( futurePose, coefs, n_section );

          if (section_change == 1)
	     {		
			//std::cerr << "consign propagating";
		n_section++;
		u_ant=0;
		calculate_consign(futurePose,coefs,n_section);

	     }
		//std::cerr << "Ref. Pose calculating..";

	reference_pose.header.frame_id = "map";
	reference_pose.pose.position.x = pose_d[0];
	reference_pose.pose.position.y = pose_d[1];
	reference_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_d[2]);
	
	reference_pose_pub.publish(reference_pose);

	pose_future.header.frame_id = "map";
	pose_future.pose.position.x = futurePose[0];
	pose_future.pose.position.y = futurePose[1];
	pose_future.pose.orientation = tf::createQuaternionMsgFromYaw(futurePose[2]);
	
	predicted_pose_pub.publish(pose_future);
	de = (futurePose[1]-pose_d[1])*cos(pose_d[2])-(futurePose[0]-pose_d[0])*sin(pose_d[2]);
	oe = Limitang(futurePose[2]-pose_d[2]);
	
	de_msg.data=de;
	oe_msg.data=oe;
	de_pub.publish(de_msg);
	oe_pub.publish(oe_msg);

	if (u>0.5) //Second half of the section, we are smoothing towards the speed of the next section
	{
		v=arrayVel[n_section]+(u-0.5)*(arrayVel[n_section+1]-arrayVel[n_section]);
	}
	else //First half of the section, we are smoothing from the speed of the previous section
	{
		v=arrayVel[n_section-1]+(u+0.5)*(arrayVel[n_section]-arrayVel[n_section-1]);
	}

	//First section
	if(n_section<3)
	{
		if (u>0.5) //Second half of the section, we are smoothing towards the speed of the next section
		{
			v=arrayVel[n_section]*(n_section+1)/3+(u-0.5)*(arrayVel[n_section+1]*(n_section+2)/3-arrayVel[n_section]*(n_section+1)/3);
		}
		else //First half of the section, we are smoothing from the speed of the previous section
		{
			v=arrayVel[n_section-1]*(n_section)/3+(u+0.5)*(arrayVel[n_section]*(n_section+1)/3-arrayVel[n_section-1]*(n_section)/3);
		}
	}

	//Last section
	else if(n_section>(n_sections-3))
	{
		if (u>0.5){ //Second half of the section, we are smoothing towards the speed of the next section
			v=arrayVel[n_section]*(n_sections-n_section)/3+(u-0.5)*(arrayVel[n_section+1]*((n_sections-n_section)-1)/3-arrayVel[n_section]*(n_sections-n_section)/3);
		}
		else //First half of the section, we are smoothing from the speed of the previous section
		{
			v=arrayVel[n_section-1]*((n_sections-n_section)+1)/3+(u+0.5)*(arrayVel[n_section]*(n_sections-n_section)/3-arrayVel[n_section-1]*((n_sections-n_section)+1)/3);
		}
	}
	if(v>v_max)
		v=v_max;
	if(v<0.001)
		stop = true;
	

	A[1]= v*sample_time;
	B[0]= v*(sample_time)+(v*v*sample_time*sample_time)/(2*dist_wheels);
	B[1]= v*(sample_time)/2;
	Dlqr(Kr,A,B,Q,R);
	Consigna_ro(de,oe,Kr,&ro);
	//std::cerr << "Velocity calculating..";
	if(stop==false){
		cmd_vel_msg.linear.x = v;
		speed_msg.data = v;
		cmd_vel_msg.angular.x = ro;
		steer_msg.data = ro/max_steer_angle;
		if (ro>max_steer_angle)
			{
			cmd_vel_msg.angular.x=max_steer_angle;
			steer_msg.data=max_steer_angle;
			}
		if (ro<-max_steer_angle)
			{
			cmd_vel_msg.angular.x=-max_steer_angle;
			steer_msg.data=-max_steer_angle;
			}
	}
	else if(stop==true)
	{
		v = 0;
		cmd_vel_msg.linear.x = 0;
		speed_msg.data = v;
		cmd_vel_msg.angular.x = 0;
		steer_msg.data = 0;
		
	}
		if(v>=0)
			std::cout << "\n Velocity -- " << v*3.6;
		cmd_vel_pub.publish(cmd_vel_msg);
		speed_pub.publish(speed_msg);
		steer_pub.publish(steer_msg);

		//vel_array.push_back(v*3.6);
		//std::cout << " Path Length : " << m << " Velocity Length : " << vel_array.size();
		//std::cout << "Count -- " << zero_cnt; 
		
	// std::cerr<< "Velocity Published" << std::endl;

	//WE SAVE SPEEDS IN THE SPEED BUFFER
	for (int i=1; i<n_ret_vel; i++)
	{
		evol_vel[0][i-1]=evol_vel[0][i];
		evol_vel[1][i-1]=evol_vel[1][i];
	}
	evol_vel[0][n_ret_vel-1] = v;
	evol_vel[1][n_ret_vel-1] = ro;
	}
	else if (manual == true || obstacle == true)
	{
		cmd_vel_msg.linear.x = 0;
		speed_msg.data = 0;
		cmd_vel_msg.angular.x = 0;
		steer_msg.data = 0;
		std::cout << "\n Velocity -- " << v*3.6;
		std::cout << "\n Vehicle Stationary ... ";
		//vel_array.push_back(v*3.6);
		//std::cout << " Path Length : " << m << " Velocity Length : " << vel_array.size();
		//std::cout << "Count -- " << zero_cnt; 
		cmd_vel_pub.publish(cmd_vel_msg);
		speed_pub.publish(speed_msg);
		steer_pub.publish(steer_msg);
	}
}

//********************************MAIN*****************************************//

int main(int argc, char **argv) 
{

	ros::init(argc, argv, "spline_lqr_node");
	ros::NodeHandle n;
	ros::Timer timer;

	n.param<int>("/controller/N_max",N_max,500);
	n.param<float>("/controller/min_dist",min_dist,0.1);
	n.param<double>("/controller/rc_max",rc_max,20);
	n.param<double>("/controller/rc_min",rc_min,1);
    n.param<double>("/controller/v_max",v_max,1);
    n.param<int>("/controller/n_ret_vel",n_ret_vel,2);
    n.param<double>("/controller/sample_time",sample_time,0.1);
	n.param<double>("/controller/q11",q11,0);
	n.param<double>("/controller/q22",q22,0);
	n.param<double>("/controller/r11",r11,0);

	ROS_INFO("\nPARAMETERS:");
	ROS_INFO("\tN_max: %d",N_max);
    ROS_INFO("\tmin_dist: %lf",min_dist);
    ROS_INFO("\trc_max: %lf",rc_max);
	ROS_INFO("\trc_min: %lf",rc_min);
 	ROS_INFO("\tv_max: %lf",v_max);
 	ROS_INFO("\tn_ret_vel: %d",n_ret_vel);
	ROS_INFO("\tsample_time: %lf",sample_time);
	ROS_INFO("\tq11: %lf",q11);
	ROS_INFO("\tq22: %lf",q22);
	ROS_INFO("\tr11: %lf",r11);

	A[0]=1.0; A[1]=v * sample_time; A[2]=0.0; A[3]=1.0;
	B[0]=v * (sample_time * sample_time)/2; B[1]=sample_time;
	Q[0]=q11; Q[1]=0.0; Q[2]=0.0; Q[3]=q22;
	R=r11;

	std::cout << " \n Waiting for Path ... ";
	std::cout << " \n Waiting for Odometry Data ... ";
	ros::Subscriber spline_sub = n.subscribe("/Trajectory", 1000, trajectoryCallback); 
	ros::Subscriber odom_sub = n.subscribe("/absolute_pose", 1000, odometryCallback); 
	ros::Subscriber manual_flag = n.subscribe("/manual",10,manualCallback);
	ros::Subscriber obstacle_flag = n.subscribe("/obstacle",10,obstacleCallback);
	//ros::Subscriber external_speed_sub = n.subscribe("/external_speed", 1000, oexternalSpeedCallback); 

	
	spline_pub = n.advertise<nav_msgs::Path> ("/spline",1, true);
	spline_points_pub = n.advertise<visualization_msgs::Marker> ("/points_spline",1,true);
	reference_pose_pub = n.advertise<geometry_msgs::PoseStamped> ("/reference_pose",1,true);
	predicted_pose_pub = n.advertise<geometry_msgs::PoseStamped> ("/predicted_pose",1,true);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist> ("/cmd_vel",1,true);
	speed_pub = n.advertise<std_msgs::Float64> ("/speed",1,true);
	steer_pub = n.advertise<std_msgs::Float64> ("/steer",1,true);
	oe_pub = n.advertise<std_msgs::Float64> ("/oe",1,true);
	de_pub = n.advertise<std_msgs::Float64> ("/de",1,true);

	

	
	for (int i=0; i<n_ret_vel; i++)
	{
		evol_vel[0][i] = 0;
		evol_vel[1][i] = 0;	
	}	

	// dynamic_reconfigure::Server<dynamic_parameters::paramsConfig> server;
  	// dynamic_reconfigure::Server<dynamic_parameters::paramsConfig>::CallbackType f;

  	
	while(ros::ok())
	{
		
		//f = boost::bind(&callback, _1, _2);
  		//server.setCallback(f);
		timer= n.createTimer(ros::Duration(0.1), timerCallback);
		ros::spin();
	}
	return 0;

}

//*******************************CALCULATE SPLINE COEFFICIENTS***************************************//

void n_spline(double *x, double *y, double *o, double nu, int m)

{  
	
	double lam[m-1], deta[m], D[m]; 


	//Derivative calculation


	lam[0]=0;
	for (int i=1; i<=(m-2); i++)
        {

		lam[i]=1/(4-lam[i-1]);
	}


	deta[0]=nu*cos(o[0]);
	for (int i=1; i<=(m-2); i++)
	{
		deta[i]=(3*(x[i+1]-x[i-1])-deta[i-1])*lam[i];
	}
	deta[m-1]=nu*cos(o[1]);



	D[m-1]=deta[m-1];
	for(int i=(m-2);i>=0;i--)
	{
		D[i]=deta[i]-lam[i]*D[i+1];
	}


	//Calculation of coefficients

	for (int i=0; i<=(m-2); i++)
	{
		coefs[i][0] = x[i];
		coefs[i][1] = D[i];
		coefs[i][2] = 3*(x[i+1]-x[i])-2*D[i]-D[i+1];
		coefs[i][3] = 2*(x[i]-x[i+1])+D[i]+D[i+1];
	}

	//Derivative calculation

	lam[0]=0;
	for (int i=1; i<=(m-2); i++)
	{
		lam[i]=1/(4-lam[i-1]);
	}

	deta[0]=nu*sin(o[0]);
	for (int i=1; i<=(m-2); i++)
	{
		deta[i]=(3*(y[i+1]-y[i-1])-deta[i-1])*lam[i];
	}
	deta[m-1]=nu*sin(o[1]);



	D[m-1]=deta[m-1];
	for(int i=(m-2);i>=0;i--)
	{
		D[i]=deta[i]-lam[i]*D[i+1];
	}

	//Calculation of coefficients

	for (int i=0; i<=(m-2); i++)
	{
		coefs[i][4] = y[i];
		coefs[i][5] = D[i];
		coefs[i][6] = 3*(y[i+1]-y[i])-2*D[i]-D[i+1];
		coefs[i][7] = 2*(y[i]-y[i+1])+D[i]+D[i+1];
	}


}

//*****************CALCULATE CONSIGN***************************//

void calculate_consign(double actualPose[3],double coefs[n_max][8],int n_section)

{
	float A,B,C,D,E,F;
	double der1_x, der1_y, der2_x, der2_y, der1, der2, aux;

	A=(3*coefs[n_section][3]*coefs[n_section][3])+(3*coefs[n_section][7]*coefs[n_section][7]);
    	B=(5*coefs[n_section][2]*coefs[n_section][3])+(5*coefs[n_section][6]*coefs[n_section][7]);
    	C=(2*coefs[n_section][2]*coefs[n_section][2])+(4*coefs[n_section][1]*coefs[n_section][3])+(2*coefs[n_section][6]*coefs[n_section][6])+(4*coefs[n_section][5]*coefs[n_section][7]);
    	D=(3*coefs[n_section][2]*coefs[n_section][1])-(3*actualPose[0]*coefs[n_section][3])+(3*coefs[n_section][0]*coefs[n_section][3])+(3*coefs[n_section][5]*coefs[n_section][6])-(3*actualPose[1]*coefs[n_section][7])+(3*coefs[n_section][4]*coefs[n_section][7]);
    	E=(coefs[n_section][1]*coefs[n_section][1])-(2*actualPose[0]*coefs[n_section][2])+(2*coefs[n_section][0]*coefs[n_section][2])+(coefs[n_section][5]*coefs[n_section][5])-(2*actualPose[1]*coefs[n_section][6])+(2*coefs[n_section][4]*coefs[n_section][6]);
    	F=(coefs[n_section][0]*coefs[n_section][1])-(actualPose[0]*coefs[n_section][1])+(coefs[n_section][4]*coefs[n_section][5])-(actualPose[1]*coefs[n_section][5]);
	
	float coef[6] = {F,E,D,C,B,A};

	u = sol_equation(coef, u_ant);
	u_ant=u;

	if(n_section==0 && u<0) u=0;

	if (u<=1)
		section_change = 0;
	else if (u>1)
		{		
		section_change = 1;
		u_ant=0;
		return;
		}

	pose_d[0]=coefs[n_section][0]+coefs[n_section][1]*u+coefs[n_section][2]*u*u+coefs[n_section][3]*u*u*u;
        pose_d[1]=coefs[n_section][4]+coefs[n_section][5]*u+coefs[n_section][6]*u*u+coefs[n_section][7]*u*u*u;
	pose_d[2]=atan2((coefs[n_section][5]+(2*coefs[n_section][6]*u)+(3*coefs[n_section][7]*u*u)),(coefs[n_section][1]+(2*coefs[n_section][2]*u)+(3*coefs[n_section][3]*u*u)));
	
        if (fabs(Limitang(pose_d[2]-actualPose[2]))>PI/2)   pose_d[2]=Limitang(pose_d[2]+PI);

 	der1_x=(3*coefs[n_section][3]*u*u)+(2*coefs[n_section][2]*u)+coefs[n_section][1];
    	der1_y=(3*coefs[n_section][7]*u*u)+(2*coefs[n_section][6]*u)+coefs[n_section][5];
    	der1=der1_y/der1_x;
    	der2_x=(6*coefs[n_section][3]*u)+(2*coefs[n_section][2]);
    	der2_y=(6*coefs[n_section][7]*u)+(2*coefs[n_section][6]);
    	der2=((der2_y*der1_x)-(der1_y*der2_x))/(der1_x*der1_x*der1_x);
    	der2=fabs(der2);
    	
	if (der2<0.001)
        	rc=rc_max;
   	else
        {
		aux=(1+der1*der1);
        	rc=sqrt(aux*aux*aux);
        	rc=rc/der2;
	}

        if (rc<rc_min)
		rc=rc_min;
    	if (rc>rc_max)
        	rc=rc_max;  


}

//******************EQUATION RESOLUTION****************//

float sol_equation(float coef[6], float x)
{
	int i,j=0,n=5;
	float emax=1e-3; //maximum value of the error made
	float p,p1;
	x=x+0.1;

	do
	{
		p=coef[n]*x+coef[n-1];
		p1=coef[n];
		for(i=n-2;i>=0;i--)
		{
			p1=p+p1*x;
			p=coef[i]+p*x;
		}
		if(fabs(p)>emax)
			x-=p/p1;
		j++;
	}
	while(fabs(p)>emax&&j<100);
	
	if (x<0)
	{
		p=1;
		j=0;
		x=x+0.2;
		do
		{
		p=coef[n]*x+coef[n-1];
		p1=coef[n];
			for(i=n-2;i>=0;i--)
			{
				p1=p+p1*x;
				p=coef[i]+p*x;
			}
			if(fabs(p)>emax)
				x-=p/p1;
		j++;
		}
		while(fabs(p)>emax&&j<100);
		
	}
	return x;
	
}

//********************ANGLE LIMIT BETWEEN + -PI****************//

double Limitang( double ang )
{
	while(ang > PI) ang-= 2*PI;
	while(ang < (-PI)) ang+=2*PI;
	return ang;
}

//************************DLQR****************************************//

void Dlqr(double K[2], double A[4], double B[2], double Q[4], double R)
{

	int num_est = 2;
	double P0[4]={0,0,0,0}, inter1[2], inter3[4];
	double transA[4], BRB[4], modif[4], inter0[1];
	int ind;

	//Precalculation of elements that do not change during the loop such as the transpose of A and B * inv (R) * B

	Transposed2(A,transA);
	mult_esc_mat(1/R,B,num_est,inter1);
	Multiply(B,inter1,num_est,1,num_est,BRB);

	//Loop to calculate P0 until it stabilizes (20 times)
	
	for(ind=0;ind<20;ind++)
	{
		//We multiply BRB * P0 and store it in inter3
		Multiply(BRB,P0,num_est,num_est,num_est,inter3);
		// We add the identity matrix with the inter3 saving it in inter3
		sum_ident(inter3,num_est);
		// Inverse of inter3 and save in modif
		Inverse2(inter3,modif);
		// We multiply modif * A and save it in inter3
		Multiply(modif,A,num_est,num_est,num_est,inter3);
		// We multiply P0 * inter3 and it is saved in modif
		Multiply(P0,inter3,num_est,num_est,num_est,modif);
		// We multiply after A * modif and it is stored in P0
		Multiply(transA,modif,num_est,num_est,num_est,P0);
		// Add Q and P0 and we have a new P0
		Add(Q,P0,num_est,num_est,P0);	
	}

	// Calculation of K

	Multiply(B,P0,1,num_est,num_est,inter1);
	Multiply(inter1,B,1,num_est,1,inter0);
	inter0[0]+=R;
	inter0[0]=1/inter0[0];
	mult_esc_mat(inter0[0],B,num_est,inter1);
	Multiply(inter1,P0,1,num_est,num_est,B);
	Multiply(B,A,1,num_est,num_est,K);


}

//*******************************MATRIX MULTIPLICATION*********************************//

void Multiply(double *origin1, double *origin2, long x, long y, long z, double *goal)
{
	long i,j,k;
	double fila;
	
	for (i=0;i<x;i++)
		for (j=0;j<z;j++)
		{
			fila=0.0;
			for(k=0;k<y;k++)
				fila+=((*(origin1+i*y+k))*(*(origin2+k*z+j)));
				*(goal+i*z+j)=fila;
		}
}

//************************SCALAR MULTIPLICATION BY MATRIX*********************************//

void mult_esc_mat(double esc, double *mat, int n, double *goal)
{
	int i;
	for(i=0;i<n;i++)
		goal[i]=mat[i]*esc;
}

//********************ADD THE IDENTITY MATRIX TO A MATRIX*********************************//

void sum_ident(double *mat, int n)
{
	double *p;
	int i;
	
	p=mat;
	for(i=0;i<n;i++)
	{
		*p=*p+1.0;
		p+=n+1;
	}
}

//*****************REVERSE OF AN ORDER MATRIX 2**********************//

void Inverse2(double *origin, double *goal)
{
	double det, inv_det;
	det=(origin[0]*origin[3])-(origin[1]*origin[2]);
	inv_det=1/det;
	goal[0]=origin[3]*inv_det;
	goal[1]=-origin[1]*inv_det;
	goal[2]=-origin[2]*inv_det;
	goal[3]=origin[0]*inv_det;
}

//******************ORDER MATRIX TRANSFER 2*************************//

void Transposed2(double *origin, double *goal)
{
	int i,j;
	for(i=0;i<2;i++)
	 for(j=0;j<2;j++)
		goal[i*2+j]=origin[j*2+i];
}

//*********************SUM OF MATRICES********************************//

void Add(double *origin1, double *origin2, int nf, int nc, double *goal)
{
	int i,j;
	for(i=0;i<2;i++)
	 for(j=0;j<2;j++)
		*(goal+i*nf+j)=(*(origin1+i*nf+j))+(*(origin2+i*nf+j));
}

//**********************APPLY THE LAW OF OPTIMAL CONTROL TO HAVE ro = (-K * x)**********************//

void Consigna_ro(double de, double oe, double Kr[2], double *ro)		
{
	double estados[2];
	estados[0]=de;
	estados[1]=oe;
	Multiply(Kr,estados,1L,2L,1L,ro);
	// Inversion of the sign of the velocity obtained
	*ro = -(*ro);
}

