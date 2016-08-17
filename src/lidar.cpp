#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <iostream>

#include <sstream>

// collect lidar data in terms of time, angle, range
typedef struct {
	double angle_min;
	double angle_max;
	double scan_time;
	double range_min;
	double range_max;
	double angle_increment;
	int ranges_num;

}LIDAR_PARAMETER;

typedef struct {
	double angle;
	double distance;
}LIDAR_RANGE;	

using namespace std;
static LIDAR_PARAMETER parameter;
static LIDAR_RANGE **range;
FILE *fp;
class SubscribeandPublish{
	public: 
		SubscribeandPublish(){
			lidar_sub = n. subscribe("scan", 1000, &SubscribeandPublish::callBack_lidar, this);
		}
	
		void callBack_lidar(const sensor_msgs::LaserScan::ConstPtr& msg){
			parameter.angle_min = msg->angle_min;
			parameter.angle_max = msg->angle_max;
			parameter.range_min = msg->range_min;
			parameter.range_max = msg->range_max;	
			parameter.angle_increment = msg->angle_increment;
			parameter.ranges_num = abs(parameter.angle_max - parameter.angle_min)/parameter.angle_increment;		
			ROS_INFO(" -- Range scanned from %f to %f, ranges size: %d", parameter.angle_min, parameter.angle_max, parameter.ranges_num);
			range = (LIDAR_RANGE**)malloc(sizeof(LIDAR_RANGE*) * parameter.ranges_num);
			
			for (int i = 0; i < parameter.ranges_num; i++){
				
				range[i] = (LIDAR_RANGE *)calloc(sizeof(LIDAR_RANGE),1);
				range[i]->angle = parameter.angle_min + i*parameter.angle_increment;		
				range[i]->distance = msg->ranges[i];
				ROS_INFO( " -- %d: At %f :  %f m...", i, range[i]->angle, range[i]->distance);
				
			}	
			
			
		}

	private:
		ros::NodeHandle n;
		ros::Subscriber lidar_sub;
};

int main (int argc, char **argv){
		ros::init(argc, argv, "lidar");
		fp = fopen("/home/ubuntu/catkin_ws/lidar_data.txt", "w");
		if(fp == NULL) {
			cout<<"ERROR: open file"<<endl;
			exit(-1);
		}
		
		SubscribeandPublish ROS_LIDAR;
		ros::Rate loop_rate(25);

		ROS_INFO(" -- Start streaming lidar data ...");
		
		std::stringstream ss;
		ros::Time time = ros::Time::now();
		ss<<"Time                 Angle (radian)    Range (meter)"<<"\n";	
		string ss_tmp = ss.str();
		char *buff;
		buff = const_cast<char*>(ss_tmp.c_str());
		fputs(buff, fp);

		while(ros::ok()){
			ros::spinOnce();
	
			// record data
			
			time = ros::Time::now();
			for (int i = 0; i <parameter.ranges_num; i++){
			std::stringstream ss;
			ss<<time<<"  "<<range[i]->angle<<"  "<<range[i]->distance<<" m..."<<"\n";
			string ss_tmp = ss.str();
			char *buff;
			buff = const_cast<char*>(ss_tmp.c_str());
			fputs(buff, fp);
			}
			loop_rate.sleep();
		}
		fclose(fp);
}
			
			
			
