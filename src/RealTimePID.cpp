//receive speed and loop number from MATLAB
#include "ros/ros.h"
#include "racecar/CMD.h"
#include "racecar/LOC.h"
#include "racecar/POINT.h"
#include "racecar/TRA.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <vector>

/* Macros for computing */

#define PI 3.14159265
#define K 200

using namespace std;

/* declaring data structure */
/* path matrix */
typedef struct {
	double x1;
	double x2;
}TRAJECTORY;
/* car state structure */
typedef struct {
	float x;
	float y;
	float heading;
	float InTemp;
	float gamma;
}STATE;
/* line constant structure */
typedef struct {
	float A;
	float B;
	float C;
}LINE_CONSTANT;

/* global variable */
int j = 0 ;
int turn = 650;
bool path_init = false;
bool loop_init = false;
bool speed_init = false;
static int speed = 0;
static int loop_num = 1;
static int pathsize = 0;
static char *buffer = NULL;
static LINE_CONSTANT *line;
static TRAJECTORY **path; //matrix holding tracjectory
static STATE *car_state; //state of the car
static float P2, I, D; //PID control variables
static float P1, error_d, error_an;
static int pError, intergral, derivative;
static float steering, steering2;

/* function header */
int PID_init();
int sign_d(double val);

/* ROS class for subscribing and publishing to different node */
class SubscribeandPublish {
public:
	/* class constructor */
	SubscribeandPublish() 
	{
		/* declare publisher and subscriber */
		pub_ = n_.advertise<racecar::CMD>("command", 1000);
		
		sub_loc = n_.subscribe("localization", 1000, &SubscribeandPublish::callBack_localization, this);
		sub_tra = n_.subscribe("trajectory", 1000,  &SubscribeandPublish::callBack_trajectory, this);
		sub_loop = n_.subscribe("loop_num", 1000,  &SubscribeandPublish::callBack_loop, this);
                sub_speed = n_.subscribe("speed", 1000,  &SubscribeandPublish::callBack_speed, this);
	}
	void callBack_loop(const std_msgs::Int32::ConstPtr& msg) {
		
		loop_num = msg->data;
		loop_init = true;

	}
        void callBack_speed(const std_msgs::Int32::ConstPtr& msg) {
		
		speed = msg->data;
		speed_init = true;

	}
	void callBack_localization(const racecar::LOC::ConstPtr& msg) 
	{

		double val = (line->A*car_state->x) + (line->B*car_state->y)+ line->C;
		/* previous location of car */
		ROS_INFO("x: %f, y: %f, head: %f", car_state->x, car_state->y, car_state->heading);
		/* create command message to publish */ 
		::racecar::CMD cmd;
		/* calculate error variable */
		error_an =  (car_state->heading -car_state->gamma);
		error_d = sign_d(val) * abs(val) /sqrt(pow(line->A,2) + pow(line->B,2));
		if (abs(error_an) > PI){
			error_an = (-1)*sign_d(error_an)*(2*PI - abs(error_an));
		}
		ROS_INFO("error_an = %f, error_d = %f", error_an, error_d);
		/* calculate initial steering angle */
		steering = P1 * error_d + P2 * error_an;
		/* calculate final steering angle */
		steering2 = 600 / (1+exp((-steering)/K)) + 1250;
		ROS_INFO("steering2 = %f, InTemp = %f", round(steering2), car_state->InTemp);
		/* send command if steering angle is different from that of the previous one */
		if (abs(round(steering2) - (car_state->InTemp)) >= 10){
			car_state->InTemp = round(steering2);			
			cmd.mode = 'f';				        
			cmd.speed = speed;   			
			cmd.steering = round(steering2);
	
			ROS_INFO("sending data: \n");
			std::cout<<"data: speed" << cmd.speed <<", steering" <<cmd.steering<<"\n";
			
			pub_.publish(cmd);
		}
		/* update car state */
		car_state->x = msg->x_cor;
		car_state->y = msg->y_cor;
		car_state->heading = msg->heading;
		ROS_INFO("updated: x: %f, y: %f, head: %f", car_state->x, car_state->y, car_state->heading); 
		
	}
        void callBack_trajectory(const racecar::TRA::ConstPtr& msg)
        {
	  ROS_INFO("Receive trajectory data");
	  pathsize = msg->trajectory.size();
	  path = (TRAJECTORY **)malloc(sizeof(TRAJECTORY *) * pathsize);
	  for(int i =0; i< pathsize; i++){
	    path[i] = (TRAJECTORY *)calloc(sizeof(TRAJECTORY), 1);
	    path[i]->x1 = msg->trajectory[i].point_x;
	    path[i]->x2 = msg->trajectory[i].point_y;
	    ROS_INFO("point %d of path: x = %f , y = %f", i, path[i]->x1, path[i]->x2);
	  }
	  path_init = true;
	}
	void stop()
	{
		::racecar::CMD cmd;
		cmd.mode ='f';
		cmd.speed = 0;
              //cmd.speed = 0;
		cmd.steering = 1550;
		ROS_INFO("Stopping the car");
		pub_.publish(cmd);
	}
private:
	/* class private variable */
	ros::NodeHandle n_; 
  	ros::Publisher pub_;
  	ros::Subscriber sub_loc;
        ros::Subscriber sub_tra;
	ros::Subscriber sub_loop;
	ros::Subscriber sub_speed;
};
/*********************************************
* function: sign_d() 
* purpose: sign difference wrapper function
* on a set of values
**********************************************/
int sign_d(double val) {
	if(val > 0) {
		return 1;
	}
	else if (val == 0) {
		return 0;
	}
	else {
		return -1;
	}
}
/*********************************************
* function: PID_init() 
* purpose: initilize and allocate global variables
* needed for PID control loop
**********************************************/
int PID_init() {
	sleep(3);
	int i;
      
	/* allocate data structure to hold car's state */
	car_state = (STATE *) calloc(sizeof(STATE),1);
	/* allocate data structure to path constant */
	line = (LINE_CONSTANT *) malloc(sizeof(LINE_CONSTANT));
	
	/* assign initial value */	
	
	if(car_state == NULL) return -1;
	car_state->x = 0;
	car_state->y = 0;
	car_state->heading = 0;
	car_state->InTemp = -1;
	car_state->gamma = 0;
	P1 = 0.27;//0.225;0.27
	P2 = 465;//500;465
	I = 0;
	D = 0;	
	pError = 0;
	error_d = 0;
	error_an = 0;
	intergral = 0;
	derivative = 0;
	return 0;
}
/*********************************************
* function: PID_close() 
* purpose: free all previously allocated dynamic memory
**********************************************/
void PID_close() {
	free(car_state);
	car_state = NULL;
	free(path);
	path = NULL;
	free(line);
	line = NULL;
}
int main (int argc, char **argv) {
	int i, k;
	ros::init(argc, argv, "RealTimePID");
	if(PID_init() != 0) {
		ROS_INFO("Failed to initialized PID controls parameters");
	}
	/* declare objects */
	SubscribeandPublish ROS_PID;
	ros::Rate loop_rate(100);
	//::racecar::CMD cmd;
	/* main control loop */
	
        do {
	  ROS_INFO("waiting for loop_num");
	  ros::spinOnce();
	  loop_rate.sleep();
	}while(loop_init == false && ros::ok());
	printf(" - Loops: %d", loop_num);
	do {
	  ROS_INFO("waiting for speed");
	  ros::spinOnce();
	  loop_rate.sleep();
	}while(speed_init == false && ros::ok());
        printf(" - Speed: %d", speed);
	sleep(2);
	
       do {
	  ROS_INFO("waiting for trajectory");
	  ros::spinOnce();
	  loop_rate.sleep();
	}while(path_init == false && ros::ok());

	

      printf("-run %d loops",loop_num);
      sleep(1);

	
      for (j=0; j<loop_num;j++){
		for(k=0; k<pathsize-1; k++) {
			ROS_INFO("HI");
			line->A = path[k]->x2 - path[k+1]->x2;
			line->B = path[k+1]->x1 - path[k]->x1;
			line->C = (path[k]->x1 * path[k+1]->x2) - (path[k+1]->x1 *path[k]->x2);
			car_state->gamma = atan2((path[k+1]->x2 -path[k]->x2), (path[k+1]->x1 - path[k]->x1));
			if (car_state->gamma < 0) {
				car_state->gamma = car_state->gamma +2 * PI;
			}
			/*
			if ( k == pathsize-2 && j == 9) {
				turn = 450;
			}*/
			while(sqrt(pow((car_state->x - path[k+1]->x1), 2) + pow((car_state->y - path[k+1]->x2), 2))>turn && ros::ok()) {
				ROS_INFO("travelling to next point");
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
      }
	while(ros::ok()){
	  ROS_PID.stop();
	  ros::spinOnce();
	  loop_rate.sleep();
          
	  }
	PID_close();
	return 0;
}	
