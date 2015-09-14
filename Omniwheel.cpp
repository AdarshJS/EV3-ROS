// Im am gonna change this code to omniwheeled bot

#include <ros.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include "ev3dev.h"
#include <chrono>
#include <ev3_services/Omniwheel.h>
#include <stdio.h>
//I Should create an srv file and add the header file here
using namespace std;
using namespace ev3dev;
using ev3_services::Omniwheel;
string ns = "/robot3/";
ros::NodeHandle  nh; 

motor left_motor, right_motor, rear_motor;
//sensor s;
/*the srv file must be of the form 

float vel
int time
---
float vl
float vr
*/
float R=0.03,speed;	
const float deg2rad = M_PI/180.0;
void svcCallback(const ev3_services::Omniwheel::Request &req , ev3_services::Omniwheel::Response &res){ // The callback function
	//float vx, wt, vl, vr; // LOCAL VARIABLES DEFINED AGAIn
	speed = R*deg2rad;
	//req.time*=1000;//to convert secs to millisecs But this line shows an error so Commented it
	res.v1=req.vel/speed; // /speed was taken out
	res.v2=-(req.vel/speed);
	res.v3=0;   
// The front two wheels are taken as v1 and v2 and the back wheen is v3	
if(res.v1!=0)
	{
		left_motor.set_pulses_per_second_setpoint(res.v1);
		left_motor.set_time_setpoint(req.time);// time is in milliseconds
		left_motor.run(true);  
	}
	else
	{
		left_motor.set_pulses_per_second_setpoint(0);
		left_motor.run(false);	
	}
	if(res.v2!=0)
	{
		right_motor.set_pulses_per_second_setpoint(res.v2); 
		right_motor.set_time_setpoint(req.time);
		right_motor.run(true);
	}
	else
	{
		right_motor.set_pulses_per_second_setpoint(0);
		right_motor.run(false);	
	}
	
	if(res.v3!=0)
	{
		rear_motor.set_pulses_per_second_setpoint(res.v3); 
		rear_motor.set_time_setpoint(req.time);
		rear_motor.run(true);
	}
	else
	{
		rear_motor.set_pulses_per_second_setpoint(0);
		rear_motor.run(false);	
	}//cout << "received " << vx << "," << wt << "=>" << vl <<"," << vr << endl;
}

ros::ServiceServer<ev3_services::Omniwheel::Request, ev3_services::Omniwheel::Response> server("Omniwheel",&svcCallback); //SERVICE DEFINITION servvice name is Omniwheel




int main(int argc, char* argv[]) 
{  

// These following lines are used to Make sure that command lline args are correct
	

if(argc<5) // if number of args are less than 5
	{
		cerr << "Usage: " << argv[0] << " <socket> <Front_left_port> <Front_right_port> <Rear_motor_port> <hz>" << endl;// Front left motor port is in port A, front right is in port B and rear motor is in port C
		return 1;
	}
	int milliseconds = 100;
	if(argc==6)
    	milliseconds = 1000/atoi(argv[5]);
    // cout<<"milliseconds"<<milliseconds;
    string left_motor_port(argv[2]);
    string right_motor_port(argv[3]);
    string rear_port(argv[4]);
    
	// TODO: Check if both are of same type

	left_motor = motor(left_motor_port); 
	right_motor = motor(right_motor_port);
	rear_motor = motor(rear_port);
	//s = sensor(sensor_port);
   /* if(s.type()!="ev3-uart-30")  // sensor object s will not be used hereafter
    {
		cerr << "Invalid sensor type. Must be EV3 ultrasonic. Given sensor is of type " << s.type() << endl;
		return 1;
	}*/    	
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

	rear_motor.reset();
	rear_motor.set_position(0);
	rear_motor.set_run_mode("time");
	rear_motor.set_stop_mode("brake");
	rear_motor.set_regulation_mode("on");

nh.initNode(argv[1]); // argv[1] is the ip address of the computer
nh.advertiseService(server);

// JUST TO KNOW IF THE NODE IS ALIVE

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg); // this publisher is just to show that the node is alive
char hello[13] = "hello world!";
nh.advertise(chatter);
str_msg.data = hello;

        while(1) {cout<<"service is being advertised. Waiting for request"<<endl;
               chatter.publish(&str_msg);
               nh.spinOnce();
               sleep(1);
        }
	
return 0;
}





