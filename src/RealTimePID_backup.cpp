#include "ros/ros.h"
#include "racecar/CMD.h"
#include "racecar/LOC.h"
#include "std_msgs/String.h"


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#define PI 3.14159265
#define PATHSIZE 4
typedef struct {
	int x1;
	int x2;
}TRAJECTORY;

typedef struct {
	float x;
	float y;
	float heading;
	float InTemp;
}STATE;
typedef struct {
	float A;
	float B;
	float C;
}LINE_CONSTANT;
/*typedef struct {
	int steering;
	int speed;
}COMMAND;*/

//static COMMAND *cmd;
static int received = 0;
static char *buffer = NULL;
static LINE_CONSTANT *line;
static TRAJECTORY **path; //matrix holding tracjectory
static STATE *car_state; //state of the car
static float P2, I, D; //PID control variables
static float P1, error_d, error_an;
static int pError, intergral, derivative;
static float steering, steering2;


int PID_init();
void get_car_state(const racecar::LOC::ConstPtr& msg);
int sign_an(STATE *car_state, float gamma);
int sign_d(LINE_CONSTANT *line, STATE *car_state, float gamma);

void get_car_state(const racecar::LOC::ConstPtr& msg) {
	car_state->x = msg->x_cor;
	car_state->y = msg->y_cor;
	car_state->heading = msg->heading;
	ROS_INFO("Received commands!");
	ROS_INFO("x: %f, y: %f, head: %f", car_state->x, car_state->y, car_state->heading);
	
}	

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
	//cmd = (COMMAND *) calloc(sizeof(COMMAND),1);
	line = (LINE_CONSTANT *) malloc(sizeof(LINE_CONSTANT));
	
	/* assign initial value */	
	
	if(car_state == NULL) return -1;

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
/*int sign_an(STATE *car_state, float gamma) {
	if((-1) * (car_state->heading -gamma) > 0) {
		return 1;
	}
	else if ((-1) * (car_state->heading -gamma) == 0) {
		return 0;
	}
	else  {
		return -1;
	}
}*/
int sign_d(LINE_CONSTANT *line, STATE *car_state, float gamma) {
	int error;
	double A_sqr = pow(line->A, 2);
	double B_sqr = pow(line->B, 2);
	double val = (line->A*car_state->x) + (line->B*car_state->y)+ line->C;
	double abs_val = abs(val);
	double sqrt_val = sqrt(A_sqr + B_sqr);
	if(((val * abs_val) / sqrt_val) > 0) {
		return 1;
	}
	else if (((val * abs_val) / sqrt_val) > 0) {
		return 0;
	}
	else {
		return -1;
	}
}

int main (int argc, char **argv) {
	int i, k, gamma;
	float val1, val2, val3;
	ros::init(argc, argv, "RealTimePID");
	if(PID_init() != 0) {
		ROS_INFO("Failed to initialized PID controls parameters");
	}
	ros::NodeHandle n;
	::racecar::CMD cmd;
	ros::Subscriber sub = n.subscribe("localization", 1000, get_car_state);
	ros::Publisher pub = n.advertise<racecar::CMD>("command", 1000);
	ros::Rate loop_rate(100);
	/* declare trajectory */
	for(k=1; k<3; k++) {
		line->A = path[k]->x2 - path[k+1]->x2;
		line->B = path[k+1]->x1 - path[k]->x1;
		line->C = path[k]->x1 * path[k+1]->x2 - path[k+1]->x1 *path[k]->x2;
		gamma = atan2((path[k+1]->x2 -path[k]->x2), (path[k+1]->x1 - path[k]->x1));
		if (gamma < 0) {
			gamma = gamma +2 * PI;
		}
		val1 = pow((car_state->x - path[k+1]->x1), 2);
	 	val2 = pow((car_state->y - path[k+1]->x2), 2);
		val3 = val1+ val2;
		while (val3 > 650 && ros::ok()) {
			error_an = (-1) * (car_state->heading -gamma);
			error_d = sign_d(line, car_state, gamma);
			steering = P1 * error_an + P2 * error_d;
			int K = 200;
			steering2 = 1000 / (1+exp((0-steering)/K)) + 1000;

			if (round(steering2) != InTemp){
				InTemp = round(steering2);
				cmd.mode = 'f';
				cmd.speed = 34;
				cmd.steering = round(steering2);
				
				ROS_INFO("sending data: \n");
				std::cout<<"data: speed" << cmd.speed <<", steering" <<cmd.steering<<"\n";
				pub.publish(cmd);
				ros::spinOnce();
				loop_rate.sleep();
				
			}	
			
			pError = error_d;
		}	
		ros::spin();
	}
	/* stop the car */
	cmd.mode = 'f';
	cmd.speed = 0;
	cmd.steering = 1500;
	pub.publish(cmd);
	ros::spinOnce();
	loop_rate.sleep();
	return 0;
}
