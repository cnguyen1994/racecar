//receive speed and loop number from MATLAB

extern "C" {
	#include <stdio.h>
	#include <stdlib.h>
	#include <unistd.h>
	#include <string.h>
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <netdb.h>
	#include <math.h>
	#include <time.h>
	#include <pthread.h>
}
#include <fstream>


#define LIBBOT_PRESENT 0
#include <sstream>
#include <iostream>
#include <ctime>
#include <iostream>
#include <vector>

#include "rrts.hpp"
#include "system_single_integrator.h"

#include "ros/ros.h"
#include "racecar/POINT.h"
#include "racecar/TRA.h"
#include "racecar/LOC.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

using namespace RRTstar;
using namespace SingleIntegrator;

using namespace std;
static int sockfd = -1;


typedef Planner<State,Trajectory,System> planner_t;
typedef Vertex<State,Trajectory,System> vertex_t;

typedef struct {
  double x,y,th;
}Position;

typedef struct {
  double x,y;
}Point;

void error(const char *msg) {
  perror(msg);
  exit(0);
};
  /***********parse string function**************/
  /* buffer: position data buffer for parsing   */
  /* buf_pos: pointer to the head of buffer     */
  /* loc: data structure to hold parsed pos     */
  /**********************************************/
//transfer buffer to loc->x,y, th
//transfer temp to buffer
bool parse_loc(char *buffer, int *buf_pos, Position *loc) {
	if (buffer[*buf_pos] == '\0')
		return false;
	char temp[16];
	int i;

	bzero(temp, 16);
	for (i = 0; buffer[*buf_pos] != ','; i ++, *buf_pos = *buf_pos + 1)
		temp[i] = buffer[*buf_pos];
	*buf_pos = *buf_pos + 1;
	loc->x = atof(temp);

	bzero(temp, 16);
	for (i = 0; buffer[*buf_pos] != ','; i ++, *buf_pos = *buf_pos + 1) {
		temp[i] = buffer[*buf_pos];
	}
	*buf_pos = *buf_pos + 1;
	loc->y = atof(temp);
	
// **************************************
	for (i = 0; buffer[*buf_pos+i] != '|' && buffer[*buf_pos+i] != '\0'; i ++);
	strncpy(temp, buffer+*buf_pos, i);// take z out
// **************************************
	bzero(temp, 16);//clear temp
	for (i = 0; buffer[*buf_pos] != '|' && buffer[*buf_pos] != '\0'; i ++, *buf_pos = *buf_pos + 1) {
		temp[i] = buffer[*buf_pos];
	}
	*buf_pos = *buf_pos + 1;
	loc->th = atof(temp);
	return true;
}
char** parse_str(char *a_str, const char delimiter) {
	char **result = 0;
	size_t count = 0;
	char* tmp = a_str;
	char* last_comma = 0;
	char delim[2];
	delim[0] = delimiter;
	delim[1] = 0;
	
	while (*tmp) {
		if (delimiter == *tmp){
			count ++;
			last_comma = tmp;
		}
		tmp++;
	}
	count += last_comma < (a_str + strlen(a_str) -1);
	count ++;
	result = (char**) malloc(sizeof(char*) * count);
	if (result){
		size_t idx =0;
		char* token = strtok(a_str, delim);
		while(token) {
			assert(idx < count);
			*(result + idx++) = strdup(token);
			token = strtok(0, delim);
		}
		assert(idx == count -1);
		*(result + idx) =0;
	}
	return result;
}
int main (int argc, char**argv) {
  
  /*-ViCon client initialization stt----------------------------------------*/

  int portno, num;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  char buffer[2048];
  char compare[8];
  
  cout<<"RRT Program has started"<<"\n";
  // port number
  portno = 30002;
  // server address
  server = gethostbyname("192.168.0.2");
  if (server == NULL) error("ERROR no such host");
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)error("ERROR opening socket");
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
  serv_addr.sin_port = htons(portno);
  if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)error("ERROR connecting");
  else printf("- ViCon server connected...\n");

  printf("- ViCon client initialization done...\n");

  sleep(2);

  /*-ViCon client initialization end*/
  


  /*-Get position of car-*/
  
  //get into the mode of getting check points
  bzero(buffer, 2048);
  strcpy(buffer, "start");//strcpy(buffer, "get_all");
  num = write(sockfd, buffer, strlen(buffer));
  printf("- done sending request\n");
  //wait for the turn
   bzero(buffer, 2048);
   num = read(sockfd, buffer, 2047);  //read villian
 

  printf("- received: %s\n", buffer);
  strncpy(compare,buffer, 8);
  printf(" -got : %s\n", compare);
  
  if (strncmp(buffer, "villian", 8) ==0){
  printf(" - Got cha!");
  }
  //send request
  bzero(buffer, 2048);
  strcpy(buffer, "get_all");
  printf("- Sending: %s\n", buffer);
  num = write(sockfd, buffer, strlen(buffer));
  printf("- request sent\n");
  
  if (num < 0)
 	error("ERROR writing to socket");
  sleep(1);
  printf("- Getting all positions...\n");
  bzero(buffer, 2048);
  printf("- Buffer cleared 1: %s\n", buffer);
  
	// TODO: increase the size of buffer to track more obstacles.

//get check points from MATLAB
  num = read(sockfd, buffer, 2048);
 if (num < 0)
       	error("ERROR reading from socket");

  Position rc_loc;
  Position goal_loc;
  Position obs_loc[200];
  int obs_count = 0;
  int buf_pos = 0;
  std_msgs::Int32 loop_num;
  std_msgs::Int32 speed;
 // char gl[256];
  printf("- Buffer read: %s\n", buffer);	
 // bzero(gl, 256);
//  strcpy(gl, "-1264,2185,213");
//  parse_loc(gl, &buf_pos, &goal_loc);
//  printf("- Goal point: [%lf, %lf, %lf]\n", goal_loc.x, goal_loc.y, goal_loc.th);	
  parse_loc(buffer, &buf_pos, &rc_loc);
  printf("- Starting point: [%lf, %lf, %lf]\n", rc_loc.x, rc_loc.y, rc_loc.th);

//check point count
   while (parse_loc(buffer, &buf_pos, obs_loc+obs_count)) {
     printf("- Obstacle point: [%lf, %lf, %lf]\n", obs_loc[obs_count].x, obs_loc[obs_count].y, obs_loc[obs_count].th);
     obs_count ++;
   }
   printf("- All %d Positions received...\n", obs_count+1);
  
  bzero(buffer,2056);
  strcpy(buffer,"position received");
  num = write(sockfd, buffer,strlen(buffer));
  

//receive loop number
   int tmp_loop = 0;
   bzero(buffer, 2056);
   num = read(sockfd, buffer, 2056);
   sscanf(buffer,"%d", &tmp_loop);
   loop_num.data = tmp_loop;
   printf(" -received loop number: %d \n", loop_num.data);

   bzero(buffer,2056);
   strcpy(buffer,"loop number received");
   num = write(sockfd, buffer,strlen(buffer));

//receive speed
   int tmp_speed = 0;
   bzero(buffer, 2056);
   num = read(sockfd, buffer, 2056);
   sscanf(buffer,"%d", &tmp_speed);
   speed.data = tmp_speed;
   printf(" -received speed: %d \n", speed.data);
   sleep(1);

/*
   bzero(buffer,2056);
   strcpy(buffer,"speed received");
   num = write(sockfd, buffer,strlen(buffer));
*/
  
//response back 
  bzero(buffer, 2056);
  strcpy(buffer, "villian_received");
  num = write(sockfd, buffer, strlen(buffer));
  if (num < 0)
 	error("ERROR writing to socket");

 /*-Get positions (racecar, obstacles) end-*/

   /*-RRT* init-*/
  
    planner_t rrts;
    
    cout << "RRTstar is alive" << endl;
    /*     
    
    // Run the algorithm for 10000 iteartions
    for (int i = 0; i < 1000; i++) {
      //cout<<"itertations: "<<i<<"\n";
        rrts.iteration ();
    }
    cout<<"All iterations finished"<<"\n";
*/
    Point chk_pnt[256];
    float chk_cnt;//int


 
 /*get check point*/
	
	chk_pnt[0].x = rc_loc.x; chk_pnt[0].y = rc_loc.y;
	for (int count = 0; count < obs_count; count ++){
	chk_pnt[count+1].x = obs_loc[count].x; 
	chk_pnt[count+1].y = obs_loc[count].y;
	//printf("-check point %d: %f, %f \n",count, chk_pnt[count].x, chk_pnt[count].y);
        }


	/*-RRT* initialization end-*/
    
    /*-set up ROS subscription-*/
    ros::init(argc, argv, "RRTstar");
    ros::NodeHandle n;
    ros::Publisher trajectory_pub = n.advertise<racecar::TRA>("trajectory", 1000);
    ros::Publisher localization_pub = n.advertise<racecar::LOC>("localization", 1000);
    ros::Publisher loop_pub = n.advertise<std_msgs::Int32>("loop_num", 10);
    ros::Publisher speed_pub = n.advertise<std_msgs::Int32>("speed", 10);
    ::racecar::LOC loc_data;
    ::racecar::TRA trajec;
    ::racecar::POINT check_pt;
    ros::Rate loop_rate(100);

    /*-Send trajectory to PID-*/
/*
    for(int i =0 ; i < chk_cnt; i++){
      check_pt.point_x = chk_pnt[i].x;
      check_pt.point_y = chk_pnt[i].y;
      trajec.trajectory.push_back(check_pt);
    }
*/
   
	  
    
   for (int i = 0; i < obs_count+1; i++){
	check_pt.point_x = chk_pnt[i].x;
	check_pt.point_y = chk_pnt[i].y;
	trajec.trajectory.push_back(check_pt);
   }

   for(int i =0; i<50; i++) {
 	ROS_INFO("publish loop number");
	loop_pub.publish(loop_num);
	ros::spinOnce();
	loop_rate.sleep();
   }
  sleep(1);
   for(int i =0; i<60; i++) {
 	ROS_INFO("publish speed");
	speed_pub.publish(speed);
	ros::spinOnce();
	loop_rate.sleep();
   }
   sleep(0.5);	
 
    for(int i =0; i < 50; i++) {
      ROS_INFO("publish trajectory");
      trajectory_pub.publish(trajec);
      ros::spinOnce();
      loop_rate.sleep();
    }
    /*-Send complete-*/
	ROS_INFO("tracking car");
    /*-Tracking Car program-*/
    	while(ros::ok()) {
		/* publishing at 10Mhz */
		//ros::Rate loop_rate(10);

                bzero(buffer, 1024);
		strcpy(buffer, "get_state2minimal");
		num = write(sockfd, buffer, strlen(buffer));
		if (num < 0)
			ROS_INFO("ERROR writing to socket");
		bzero(buffer, 1024);
		num = read(sockfd, buffer, 1024);
		if (num < 0) {
			ROS_INFO("ERROR reading from socket");	
		}
		char **tokens = parse_str(buffer, ',');
	        loc_data.x_cor = atof(*(tokens));
		loc_data.y_cor = atof(*(tokens + 1));
		loc_data.heading = atof(*(tokens +2));
		ROS_INFO("publishing: x: %f, y: %f, heading: %f", loc_data.x_cor, loc_data.y_cor, loc_data.heading);
		localization_pub.publish(loc_data);
		ros::spinOnce();
		loop_rate.sleep();
	}
    close(sockfd);
    sockfd =-1;
    return 0;

    
}

