#include <ros.h>
#include <geometry_msgs/PoseStamped.h>
//#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include "ev3dev.h"
#include <chrono>
//#include <ev3_services/Gostraight.h>
#include <ev3_services/Trace_arc.h>
#include <stdio.h>
//I should create an srv file and add the header file here
using namespace std;
using namespace ev3dev;
//using ev3_services::Gostraight;
using ev3_services::Trace_arc;
string ns = "/robot3/";
ros::NodeHandle  nh; 

motor left_motor, right_motor;
sensor s;
/*the srv file must be of the form 

float vel
int time
---
float vl
float vr
*/

//Global variable declarartion
float x = 0.0, y = 0.0, t = 0.0;
float x_est = 0.0, y_est = 0.0, t_est = 0.0;
bool new_estimate = false;
float R=0.03,L=0.12,speed;	
const float deg2rad = M_PI/180.0;
float vx, wt, vl, vr;
bool running = true;
int encoder[2]={0,0}, prev_encoder[2] = {0,0}, dl, dr;
const char map_name[4] = "map";


/*void svcCallback_Gostraight(const ev3_services::Gostraight::Request &req , ev3_services::Gostraight::Response &res){ // The callback function
	//float vx, wt, vl, vr; // LOCAL VARIABLES DEFINED AGAIn
	speed = R*deg2rad;
	//req.time*=1000;//to convert secs to millisecs But this line shows an error so Commented it
	res.vl=req.vel/speed; // /speed was taken out
	res.vr=req.vel/speed;   
	if(res.vl!=0)
	{
		left_motor.set_pulses_per_second_setpoint(res.vl);
		left_motor.set_time_setpoint(req.time);// time is in milliseconds
		left_motor.run(true);  
	}
	else
	{
		left_motor.set_pulses_per_second_setpoint(0);
		left_motor.run(false);	
	}
	if(res.vr!=0)
	{
		right_motor.set_pulses_per_second_setpoint(res.vr); 
		right_motor.set_time_setpoint(req.time);
		right_motor.run(true);
	}
	else
	{
		right_motor.set_pulses_per_second_setpoint(0);
		right_motor.run(false);	
	}
	//cout << "received " << vx << "," << wt << "=>" << vl <<"," << vr << endl;
}*/

//----------------------------------------------------------------------------------------------------------------------------------------
void svcCallback_Trace_arc(const ev3_services::Trace_arc::Request &req , ev3_services::Trace_arc::Response &res){ // The callback function
	//float vx, wt, vl, vr; // LOCAL VARIABLES DEFINED AGAIn
	speed = R*deg2rad;
	//req.time*=1000;//to convert secs to millisecs But this line shows an error so Commented it
	res.vl= (req.vx+L/2*req.wt)/speed; // /speed was taken out
	res.vr= (req.vx-L/2*req.wt)/speed;   
	if(res.vl!=0)
	{
		left_motor.set_pulses_per_second_setpoint(res.vl);
		left_motor.set_time_setpoint(req.time);// time is in milliseconds
		left_motor.run(true);  
	}
	else
	{
		left_motor.set_pulses_per_second_setpoint(0);
		left_motor.run(false);	
	}
	if(res.vr!=0)
	{
		right_motor.set_pulses_per_second_setpoint(res.vr); 
		right_motor.set_time_setpoint(req.time);
		right_motor.run(true);
	}
	else
	{
		right_motor.set_pulses_per_second_setpoint(0);
		right_motor.run(false);	
	}
	//cout << "received " << vx << "," << wt << "=>" << vl <<"," << vr << endl;
}
//------------------------------------------------------------------------------------------------------------------------------------------


void pose_cb(const geometry_msgs::PoseStamped& msg)
{ // this is the callback function for a topic published by contour.cpp
	x_est = msg.pose.position.x;  

	y_est = msg.pose.position.y;  
	geometry_msgs::Quaternion q = msg.pose.orientation;
	t_est = 2.0*atan2(q.z,q.w); //related to quaternions, t is actually not time it is theta
	new_estimate = true; // only if there is a change in position... 
	cout << " Estimate: " << x_est << "," << y_est << "," << t_est <<endl;
}




//ros::ServiceServer<ev3_services::Gostraight::Request, ev3_services::Gostraight::Response> server1("Gostraight",&svcCallback_Gostraight); //SERVICE DEFINITION servvice name is Gostraight
ros::ServiceServer<ev3_services::Trace_arc::Request, ev3_services::Trace_arc::Response> server2("Trace_arc",&svcCallback_Trace_arc);
ros::Subscriber<geometry_msgs::PoseStamped> pose_sub("/robot3/pose_estimate" , pose_cb );

void odometry()
{
	int encoder[2]={0,0}, prev_encoder[2] = {0,0}, dl, dr; // dl is change in left motor and dr is change in right motor
		if(new_estimate)
		{ // if new_estimate is true
			x = x_est;
			y = y_est;
			t = t_est;
			new_estimate = false;
		}
		encoder[0] = left_motor.position();
		encoder[1] = right_motor.position();
		dl = encoder[0]-prev_encoder[0];
		dr = encoder[1]-prev_encoder[1]; // current position- prev position
		x += cos(t)*speed*(dl+dr)/2.0;
		y += sin(t)*speed*(dl+dr)/2.0;  // avg. of dl and dr are takin i guess
		t += speed*(dl-dr)/L; // t is theta          
		prev_encoder[0] = encoder[0];
		prev_encoder[1] = encoder[1];		
		vl = left_motor.pulses_per_second(); // this function returns the velocity. 
		vr = right_motor.pulses_per_second();
		
		vx = (vl+vr)/2*speed; 
		wt = (vl-vr)/L*speed; 
		
	}



int main(int argc, char* argv[]) 
{  

// These following lines are used to Make sure that command lline args are correct
	

if(argc<5) // if number of args are less than 5
	{
		cerr << "Usage: " << argv[0] << " <socket> <left_motor_port> <right_motor_port> <sensor_port> <hz>" << endl;
		return 1;
	}
	int milliseconds = 100;
	if(argc==6)
    	milliseconds = 1000/atoi(argv[5]);
    // cout<<"milliseconds"<<milliseconds;
    string left_motor_port(argv[2]);
    string right_motor_port(argv[3]);
    string sensor_port(argv[4]);
    // if(left_motor_port<1||left_motor_port>4||right_motor_port<1||right_motor_port>4||left_motor_port==right_motor_port)
    // {
		// cerr << "Invalid motor port numbers. Must be 1, 2, 3 or 4 and distinct." << endl;
		// return 1;
	// }

	// TODO: Check if both are of same type

	left_motor = motor(left_motor_port); 
	right_motor = motor(right_motor_port);
	s = sensor(sensor_port);
    if(s.type()!="ev3-uart-30")  // sensor object s will not be used hereafter
    {
		cerr << "Invalid sensor type. Must be EV3 ultrasonic. Given sensor is of type " << s.type() << endl;
		return 1;
	}    	
//---------------------------------------------------------------------------------------------------------------------------------------------
	// TODO: Check if both were initialised

	left_motor.reset();
	left_motor.set_position(0);
	left_motor.set_run_mode("time");// changed from forever mode to time
	left_motor.set_stop_mode("brake");
	left_motor.set_regulation_mode("on");

	right_motor.reset();
	right_motor.set_position(0);
	right_motor.set_run_mode("time");
	right_motor.set_stop_mode("brake");
	right_motor.set_regulation_mode("on");


nav_msgs::Odometry odom_msg;// msg object for the publisher
ros::Publisher odom_pub("/robot3/odom"/*topic name*/, &odom_msg);
nh.advertise(odom_pub); // advertises that odom_pub is the publisher name
tf::TransformBroadcaster odom_broadcaster; //broadcaster msg for the odometry
tf::TransformBroadcaster scan_broadcaster; // broadcaster for the ultrasonic sensor
nh.subscribe(pose_sub); 

ros::Time current_time, last_time;
	current_time = nh.now();
	last_time = nh.now();


nh.initNode(argv[1]);
odom_broadcaster.init(nh);
//nh.advertiseService(server1);
nh.advertiseService(server2);
while(!nh.connected()) {nh.spinOnce();}

// JUST TO KNOW IF THE NODE IS ALIVE
cout<<"1. Gostraight service\n2.Trace an arc service"<<endl;


        while(1) {
		             // check for incoming messages
		current_time = nh.now();
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(t); // odom_quat stores Quaternion cretaed from yaw

		// cout<<" Calculated quat: "<<endl;

		//first, we'll publish the transform over tf
		
		geometry_msgs::TransformStamped odom_trans;//message object
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = map_name;//map_name is just given as map
		const char base_link_name[18] = "/robot3/base_link";
		odom_trans.child_frame_id = base_link_name;

		// cout<<" constructed header"<<endl;
		
		// loading the message object with data
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;// as we are dealing with xy plane
		odom_trans.transform.rotation = odom_quat;// this is quaternion created from yaw angle t

		// cout<<" Constructed full message"<<endl;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		odom_msg.header.stamp = current_time;
		odom_msg.header.frame_id = map_name;

		//set the position
		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = odom_quat;

		//set the velocity
		odom_msg.child_frame_id = base_link_name;
		odom_msg.twist.twist.linear.x = vx;
		odom_msg.twist.twist.linear.y = 0;
		odom_msg.twist.twist.angular.z = wt;

		// cout<<" sizeof "<<sizeof(odom_msg)<<endl;
		//publish the message
		odom_pub.publish(&odom_msg);


		cout<<"Services are being advertised. Waiting for request"<<endl;
                
                nh.spinOnce();
                sleep(1);
        }

return 0;	
}





