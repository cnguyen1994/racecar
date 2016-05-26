#include "ros/ros.h"
#include "racecar/CMD.h"
#include "racecar/LOC.h"
#include "std_msgs/String.h"


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

/* Macros for computing */

#define PI 3.14159265
#define PATHSIZE 4
#define K 200

using namespace std;

/* declaring data structure */
/* path matrix */
typedef struct {
	int x1;
	int x2;
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
		sub_ = n_.subscribe("localization", 1000, &SubscribeandPublish::callBack, this);
	}
	void callBack(const racecar::LOC::ConstPtr& msg) 
	{

		double val = (line->A*car_state->x) + (line->B*car_state->y)+ line->C;
		/* previous location of car */
		ROS_INFO("x: %f, y: %f, head: %f", car_state->x, car_state->y, car_state->heading);
		/* create command message to publish */ 
		::racecar::CMD cmd;
		/* calculate error variable */
		error_an = (-1) * (car_state->heading -car_state->gamma);
		error_d = sign_d(val) * abs(val) / sqrt(pow(line->A,2) + pow(line->B,2));
		ROS_INFO("error_an = %f, error_d = %f", error_an, error_d);
		/* calculate initial steering angle */
		steering = P1 * error_d + P2 * error_an;
		/* calculate final steering angle */
		steering2 = 1000 / (1+exp((-steering)/K)) + 1000;
		ROS_INFO("steering2 = %f, InTemp = %f", round(steering2), car_state->InTemp);
		/* send command if steering angle is different from that of the previous one */
		if (round(steering2) != car_state->InTemp){
			car_state->InTemp = round(steering2);
			cmd.mode = 'f';
			cmd.speed = 36;
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
private:
	/* class private variable */
	ros::NodeHandle n_; 
  	ros::Publisher pub_;
  	ros::Subscriber sub_;
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
	/* allocate data structure to hold trajectory information */
	path = (TRAJECTORY **) malloc(sizeof(TRAJECTORY *) * PATHSIZE);
	if(path == NULL) return -1;
	for(i =0; i<PATHSIZE; i++) {
		path[i] = (TRAJECTORY *) malloc(sizeof(TRAJECTORY));
	}
	/* declare trajectory */
	path[0]->x1 = 1173; path[0]->x2 = -3065;
	path[1]->x1 = -322; path[1]->x2 = -861;
	path[2]->x1 = -430; path[2]->x2 = 593;
	path[3]->x1 = -1519; path[3]->x2 = 2208;
	/* allocate data structure to hold car's state */
	car_state = (STATE *) calloc(sizeof(STATE),1);
	/* allocate data structure to path constant */
	line = (LINE_CONSTANT *) malloc(sizeof(LINE_CONSTANT));
	
	/* assign initial value */	
	
	if(car_state == NULL) return -1;
	car_state->x = 1.9758841;
	car_state->y = -2.808184;
	car_state->heading = 5.125469;
	car_state->InTemp = -1;
	car_state->gamma = 0;
	P1 = 0.225;
	P2 = 500;
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
	SubscribeandPublish SAPObject;
	ros::Rate loop_rate(10);
	::racecar::CMD cmd;
	/* main control loop */
	for(k=0; k<3; k++) {
		line->A = path[k]->x2 - path[k+1]->x2;
		line->B = path[k+1]->x1 - path[k]->x1;
		line->C = (path[k]->x1 * path[k+1]->x2) - (path[k+1]->x1 *path[k]->x2);
		car_state->gamma = atan2((path[k+1]->x2 -path[k]->x2), (path[k+1]->x1 - path[k]->x1));
		if (car_state->gamma < 0) {
			car_state->gamma = car_state->gamma +2 * PI;
		}
		while(sqrt(pow((car_state->x - path[k+1]->x1), 2) + pow((car_state->y - path[k+1]->x2), 2))>450 && ros::ok()) {
			ROS_INFO("travelling to next point");
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	PID_close();
	return 0;
}	
