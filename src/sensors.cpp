#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "custom_msgs/sensorSample.h"
#include "custom_msgs/ImuSensorSample.h"
#include "custom_msgs/Internal.h"


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <iomanip>

// collect imu data in terms of time, yaw, roll, pitch, dqx, dqy, dqz, dvx, dvy, dvz
//store at "/home/ubuntu/catkin_ws/imu_data.txt"

typedef struct {
	double pitch;
	double roll;
	double yaw;
}orientation;

typedef struct {
	double dqx;
	double dqy;
	double dqz;
}angular_velocity;

typedef struct {
	double dvx;
	double dvy;
	double dvz;
}linear_acceleration;

using namespace std;
static orientation ori;
static angular_velocity an_vel;
static linear_acceleration acc;
FILE *fp;

class SubscribeandPublish{
public:
	SubscribeandPublish(){

		orient_sub = n.subscribe("mti/sensor/imu", 1000, &SubscribeandPublish::callBack_imu, this);
		imu_sub = n.subscribe("mti/sensor/sample", 1000, &SubscribeandPublish::callBack_ss, this);
	}
	
	void callBack_imu(const sensor_msgs::Imu::ConstPtr& msg){
		ori.pitch = msg->orientation.x;
		ori.roll = msg->orientation.y;
		ori.yaw = msg->orientation.z;
		
	}

	void callBack_ss(const custom_msgs::sensorSample::ConstPtr& msg){
		an_vel.dqx = msg->internal.imu.dq.x;
		an_vel.dqy = msg->internal.imu.dq.y;
		an_vel.dqz = msg->internal.imu.dq.z;
		acc.dvx = msg->internal.imu.dv.x;
		acc.dvy = msg->internal.imu.dv.y;
		acc.dvz = msg->internal.imu.dv.z;
		
	}

private:
	ros::NodeHandle n;
	ros::Subscriber orient_sub;
	ros::Subscriber imu_sub;
};

int main (int argc, char **argv){
	ros::init(argc, argv, "sensors");
	fp = fopen("/home/ubuntu/catkin_ws/imu_data.txt", "w");

	SubscribeandPublish ROS_SENSORS;
	ros::Rate loop_rate(25);
	

	ROS_INFO(" -- Start streaming sensor data...");

	// record data
	std::stringstream ss;
	ros::Time time = ros::Time::now();
	ss<<"Time                 pitch      roll     yaw     omegaX     omegaY     omegaZ     deltaX     deltaY     deltaZ"<<"\n";
	
	string ss_tmp = ss.str();
	char *buff;
	buff = const_cast<char*>(ss_tmp.c_str());
	fputs(buff, fp);
	
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();	
		ROS_INFO(" -- Received: pitch: %f, roll: %f, yaw: %f", ori.pitch, ori.roll, ori.yaw);
		ROS_INFO(" -- Angular velocity: omegaX: %f, omegaY: %f, omegaZ: %f", an_vel.dqx, an_vel.dqy, an_vel.dqz);
		ROS_INFO(" -- Linear acceleration: deltaX: %f, deltaY: %f, deltaZ: %f", acc.dvx, acc.dvy, acc.dvz);
		std::cout<<time<<"\n";

		// record data
		std::stringstream ss;
		time = ros::Time::now();
		ss<<time<<"  "<<ori.pitch<<"  "<<ori.roll<<"  "<<ori.yaw<<"  "<<an_vel.dqx<<"  "<<an_vel.dqy<<"  "<<an_vel.dqz<<"  "<<acc.dvx<<"  "<<acc.dvy<<"  "<<acc.dvz<<"\n";
		string ss_tmp = ss.str();		
		buff = const_cast<char*>(ss_tmp.c_str());
		fputs(buff, fp);
	}
}
