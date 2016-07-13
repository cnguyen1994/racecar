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
  /*if(argc != 2){
    printf("Usage: ./racecar <goal_x, goal_y, goal_z> \n");
    exit(-1);
  }*/
  /* declare goal region size */
/*  
  Position gr_size;
  gr_size.x = 200;
  gr_size.y = 200;
*/  
  /* declare region operating size */
/*
  Position ro_size;
  ro_size.x = 10000;
  ro_size.y = 10000;
*/  
  /*-ViCon client initialization stt----------------------------------------*/

  int portno, num;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  char buffer[2048];
  char compare[8];
  
  cout<<"RRT Program has started"<<"\n";
  // port number
  portno = 30001;
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
    
    
    // Create the dynamical system
    System system;
    
    // Three dimensional configuration space
    system.setNumDimensions (2);
    
    // Define the operating region
    system.regionOperating.setNumDimensions(2);
    system.regionOperating.center[0] = 0.0;
    system.regionOperating.center[1] = 0.0;
  
    system.regionOperating.size[0] = ro_size.x;
    system.regionOperating.size[1] = ro_size.y;
    
    
    // Define the goal region
    system.regionGoal.setNumDimensions(2);
    system.regionGoal.center[0] = goal_loc.x;
    system.regionGoal.center[1] = goal_loc.y;
 
    system.regionGoal.size[0] = gr_size.x;
    system.regionGoal.size[1] = gr_size.y;
    
    
    
   // Define the obstacle region
   region *obstacle;

   for (int i = 0; i < obs_count; i ++) {
     obstacle = new region;
     obstacle->setNumDimensions(2);

     obstacle->center[0] = obs_loc[i].x;
     obstacle->center[1] = obs_loc[i].y;
     obstacle->size[0] = 300;
     obstacle->size[1] = 200;
     system.obstacles.push_front (obstacle);  // Add the obstacle to the list
   } 
    

    

    // Add the system to the planner
    rrts.setSystem (system);
    
    // Set up the root vertex
    vertex_t &root = rrts.getRootVertex();  
    State &rootState = root.getState();
    rootState[0] = rc_loc.x;
    rootState[1] = rc_loc.y;
    
    
    // Initialize the planner
    rrts.initialize ();
    
    // This parameter should be larger than 1.5 for asymptotic 
    //   optimality. Larger values will weigh on optimization 
    //   rather than exploration in the RRT* algorithm. Lower 
    //   values, such as 0.1, should recover the RRT.
    rrts.setGamma (2.5);

    
    
    // Run the algorithm for 10000 iteartions
    for (int i = 0; i < 1000; i++) {
      //cout<<"itertations: "<<i<<"\n";
        rrts.iteration ();
    }
    cout<<"All iterations finished"<<"\n";
*/
    Point chk_pnt[256];
    float chk_cnt;//int
/*    int even = 0;

    list<double*> trajectory;
    if (rrts.getBestTrajectory(trajectory)) {
      list<double*>::const_iterator iterator;
      for (iterator = trajectory.begin(); iterator != trajectory.end(); iterator ++) {
	if (even % 2 == 0) {
	  chk_pnt[chk_cnt].x = *(*iterator);
	  chk_pnt[chk_cnt].y = *(*iterator+1);
	  chk_cnt ++;
	}
	even ++;
      }
      cout << "- Best trajectory found..." << endl;
      cout << "List size: " << chk_cnt << endl;
      for (int i = 0; i < chk_cnt; i ++) {
	cout << chk_pnt[i].x << ", " << chk_pnt[i].y << endl;
      }
    }
    else{
      cout << "- Trajectory not found..." << endl;
    }
*/


/* patroling path
	chk_pnt[0].x = 974; chk_pnt[0].y = -3597;	
	chk_pnt[1].x = 515; chk_pnt[1].y = -3510;
	chk_pnt[2].x = 109; chk_pnt[2].y = -3278;
	chk_pnt[3].x = -127; chk_pnt[3].y = -2952;
	chk_pnt[4].x = -158; chk_pnt[4].y = -2475;
	chk_pnt[5].x = -152; chk_pnt[5].y = -882;
	chk_pnt[6].x = -158; chk_pnt[6].y = 40.95;
	chk_pnt[7].x = -154; chk_pnt[7].y = 970.2;
	chk_pnt[8].x = -141; chk_pnt[8].y = 1222;
	chk_pnt[9].x = -22; chk_pnt[9].y = 1527;
	chk_pnt[10].x = 313; chk_pnt[10].y = 1797;
	chk_pnt[11].x = 861; chk_pnt[11].y = 1969;
	chk_pnt[12].x = 1392; chk_pnt[12].y = 1801;
	chk_pnt[13].x = 1762; chk_pnt[13].y = 1571;
	chk_pnt[14].x = 1880; chk_pnt[14].y = 1225;
	chk_pnt[15].x = 1902; chk_pnt[15].y = 598;
	chk_pnt[16].x = 1913; chk_pnt[16].y = 74;
	chk_pnt[17].x = 1907; chk_pnt[17].y = -892;
	chk_pnt[18].x = 1954; chk_pnt[18].y = -2432;
	chk_pnt[19].x = 1980; chk_pnt[19].y = -2926;
	chk_pnt[20].x = 1751; chk_pnt[20].y = -3254;
	chk_pnt[21].x = 1423; chk_pnt[21].y = -3470;
	chk_pnt[22].x = 974; chk_pnt[22].y = -3597;
*/    
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
	loop_pub.publish(loop_num);
	ros::spinOnce();
	loop_rate.sleep();
}
	
 
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

