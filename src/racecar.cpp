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
#include <sstream>
#include <iostream>
#include <iterator>
#include "src/rrts.hpp"
#include "src/system_single_integrator.h"
#include "src/rrts.h"
#include "src/system.h"
#include "src/kdtree.h"

/* RRT* dependence */
#define LIBBOT_PRESENT 0

using namespace RRTstar;
using namespace SingleIntegrator;
using namespace std;

static int sockfd = -1;
//using namespace qrk;

/* RRT* variables */

typedef Planner<State, Trajectory, System> planner_t;
typedef Vertex<State, Trajectory, System>vertex_t;

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
	strncpy(temp, buffer+*buf_pos, i);
// **************************************
	bzero(temp, 16);
	for (i = 0; buffer[*buf_pos] != '|' && buffer[*buf_pos] != '\0'; i ++, *buf_pos = *buf_pos + 1) {
		temp[i] = buffer[*buf_pos];
	}
	*buf_pos = *buf_pos + 1;
	loc->th = atof(temp);
	return true;
}

int main (int argc, char**argv){
  if(argc != 2){
    printf("Usage: ./racecar <goal_x, goal_y, goal_z> \n");
    exit(-1);
  }
  /* declare goal region size */
  Position gr_size;
  gr_size.x = 0.10;
  gr_size.y = 0.10;
  
  /* declare region operating size */
  Position ro_size;
  ro_size.x = 3.0;
  ro_size.y = 3.0;
  
  /*-ViCon client initialization stt----------------------------------------*/
  int portno, num;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  char buffer[1024];

  // port number
  portno = 3000;
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
  bzero(buffer, 1024);
  strcpy(buffer, "get_all");
  num = write(sockfd, buffer, strlen(buffer));
  if (num < 0)
 	error("ERROR writing to socket");
  printf("- Getting all positions...\n");
  bzero(buffer, 1024);
	// TODO: increase the size of buffer to track more obstacles.
  num = read(sockfd, buffer, 1023);
  if (num < 0)
       	error("ERROR reading from socket");
  Position rc_loc;
  Position goal_loc;
  Position obs_loc[8];
  int obs_count = 0;
  int buf_pos = 0;
  
  printf("- Buffer read: %s\n", buffer);	

  parse_loc(argv[1], &buf_pos, &goal_loc);
  printf("- Goal point: [%lf, %lf, %lf]\n", goal_loc.x, goal_loc.y, goal_loc.th);
	
  buf_pos = 0;
  parse_loc(buffer, &buf_pos, &rc_loc);
  printf("- Starting point: [%lf, %lf, %lf]\n", rc_loc.x, rc_loc.y, rc_loc.th);

   while (parse_loc(buffer, &buf_pos, obs_loc+obs_count)) {
     printf("- Obstacle point: [%lf, %lf, %lf]\n", obs_loc[obs_count].x, obs_loc[obs_count].y, obs_loc[obs_count].th);
     obs_count ++;
   }

   printf("- All %d Positions received...\n", obs_count+1);
   /*-Get positions (racecar, obstacles) end-*/
   /*-RRT* init-*/
   
   planner_t rrts;
   
   cout << "- RRTstar is alive..." << endl;

   // Create the dynamical system
   System system;
   // Dimensional configuration space
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
     obstacle->size[0] = 1;
     obstacle->size[1] = 0.2;
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
    rrts.setGamma (1.2);
    // Run RRT* for 2000 iterations
    for (int i = 0; i < 20000; i++){
        rrts.iteration ();
    }
    Point chk_pnt[256];
    int chk_cnt;
    int even = 0;

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
    /*-RRT* initialization end-*/
    return 0;
}
