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
#include <assert.h>
} 
#include <iostream>
#include <vector>
#include <list>
#include <ANN/ANN.h>
#include <string>
#include <sstream>
#include <fstream>
//ros
#include "ros/ros.h"
#include "racecar/CMD.h"

#define dim 2
#define k 1
#define PI 3.14159265
using namespace std;

typedef struct {
  float x;
  float y;
}POINT; 


typedef struct {
  int *nodes;
}CONNECT;

typedef struct {
  double x,y,th;
}Position;

typedef struct {
	float x;
	float y;
	float heading;
	float InTemp;
	float gamma;
}STATE;

//static GRAPH **G;
static float *J;
static CONNECT **connectivity;
static STATE *car_state; //state of the car

static int vertex_size = 0;
static int sockfd=-1;

int find_min(float *distance, int *elements, int size);
void error(const char *msg) {
  perror(msg);
  exit(0);
};

int find_min(float *Distance, int *elements, int size) {
  int node = elements[1];
  float small = Distance[node-1];
  int aha = 1;
  for(int i = 2; i<=size; i++) {
    node = elements[i];
    if(Distance[node-1] < small) {
	small = Distance[node-1];
	aha = i;
    }
  }
  return aha;
}

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
bool readPt(istream &in, ANNpoint p)			// read point (false on EOF)
{
	for (int i = 0; i < dim; i++) {
		if(!(in >> p[i])) return false;
	}
	return true;
}

void printPt(ostream &out, ANNpoint p)			// print point
{
	out << "(" << p[0];
	for (int i = 1; i < dim; i++) {
		out << ", " << p[i];
	}
	out << ")\n";
}
int main(int argc, char **argv) {

	  int portno, num;
	  struct sockaddr_in serv_addr;
	  struct hostent *server;
	  char buffer[2056];
	  float error_an, steering;
	car_state = (STATE *) calloc(sizeof(STATE),1);
	  cout<<"RRG Program has started"<<"\n";
	  // port number
	  portno = 30000;
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

	  /*-ViCon client initialization end*
	
	  /*-Get graph-*/
	bzero(buffer, 2056);
	strcpy(buffer, "get_size");
	num = write(sockfd, buffer, strlen(buffer));
	if (num < 0)
	       error("ERROR writing to socket");
	printf("- done sending request\n");
	bzero(buffer, 2056);
	num = read(sockfd, buffer, 2055);
	if (num < 0)
	 	error("ERROR reading from socket");  
	printf("-received data-");
	sscanf(buffer, "%d", &vertex_size);
	cout<<"the size of Graph is "<<vertex_size<<endl;
	
	int nPts = 0;
	ANNpointArray dataPts;
	dataPts = annAllocPts(vertex_size, 2); 
	J = (float *) calloc(sizeof(float*), (vertex_size));
	connectivity = (CONNECT**) malloc(sizeof(CONNECT*) * (vertex_size));

	ifstream infile("/home/ubuntu/catkin_ws/rrg_node.dat");
	while(infile) {
		string x;
		string y;
		ANNpoint tmp = annAllocPt(dim);
		while(getline(infile, x, ',') && nPts < vertex_size) {
			tmp[0] = (float)atof(x.c_str());
			getline(infile, y);
			tmp[1] = (float)atof(y.c_str());
			dataPts[nPts] = annCopyPt(dim, tmp);
			
			nPts++;
		}
	}	
	int i =0;
	ifstream infile_2("/home/ubuntu/catkin_ws/rrg_j.dat");	
	while(infile_2) {
		string dist;
		while(getline(infile_2, dist) && i < vertex_size) {
			J[i] = (float)atof(dist.c_str());
			
			//cout<<"node "<<i<<" distance to goal is "<<J[i]<<endl;
			i++;
		}
	}
	i = 0;
	ifstream infile_3("/home/ubuntu/catkin_ws/rrg_connect.dat");
	for(int n = 0; n<vertex_size; n++) {
		connectivity[n] = (CONNECT *) malloc(sizeof(CONNECT));
		string node_connect;
		bzero(buffer, 2056);
		strcpy(buffer, "get_size_node");
		num = write(sockfd, buffer, strlen(buffer));
		if(num <0)error("ERROR: writing to socket");
		bzero(buffer, 2056);
		num = read(sockfd, buffer, 2055);
		if(num<0)error("ERROR: reading from socket");
		int size_node = 0;
		sscanf(buffer, "%d", &size_node);
		connectivity[n]->nodes = (int *) malloc(sizeof(int) * (size_node+1));
		connectivity[n]->nodes[0] = size_node;
		for(int j =1; j<=size_node; j++) {
			getline(infile_3, node_connect, ',');
			connectivity[n]->nodes[j] = (int)atoi(node_connect.c_str());
		}
	}
		

			 	
	//convert to data into eigen vector and matrix
	ANNkd_tree* kdTree;
	ANNidxArray nnIdx;
	ANNdistArray dists;
	ANNpoint queryPt;
	ANNpoint nearestPt;
	nnIdx = new ANNidx[k];
	dists = new ANNdist[k];
	queryPt = annAllocPt(dim);
	nearestPt = annAllocPt(dim);
	kdTree = new ANNkd_tree(dataPts, vertex_size, dim);
	int nearest_node;
	int idx, next_way_point;
	ANNpoint currentPt;
	vector<int> best_neighbor;
	currentPt= annAllocPt(dim);
	car_state->InTemp = -1000;
	//ros initilization 
	ros::init(argc, argv, "RRG");
	ros::NodeHandle n; 
  	ros::Publisher pub;
	pub =  n.advertise<racecar::CMD>("command", 1000);
	ros::Rate loop_rate(50);
	::racecar::CMD cmd;
	cout<<"ROS INIT"<<endl;
	do{
		//get current location of the car from server
		bzero(buffer, 2056);
		strcpy(buffer, "get_state2minimal");
		num = write(sockfd, buffer, strlen(buffer));
		if (num < 0) ROS_INFO("ERROR writing to socket");
		bzero(buffer, 2056);
		num = read(sockfd, buffer, 2055);
		if (num < 0) ROS_INFO("ERROR reading from socket");
		//parse string	
		char **tokens = parse_str(buffer, ',');
		queryPt[0] = atof(*(tokens));
		queryPt[1] = atof(*(tokens+1));
		car_state->heading = atof(*(tokens+2));
		currentPt = annCopyPt(dim, queryPt);
		//print car location info
		ROS_INFO("x: %f, y: %f, head: %f", queryPt[0], queryPt[1], car_state->heading);
		
		//find node on graph and calculate the next point
		//nearest neighbor search based upon the current location
		kdTree->annkSearch(queryPt, k, nnIdx, dists, 0);
		//nearest node
		nearest_node = nnIdx[0]; 
		cout<<"Nearest node = "<<nearest_node<<endl;
		cout<<"distance to nearest node "<<sqrt(dists[0])<<endl; 
		//find the node of next point
		idx = find_min(J, connectivity[nearest_node]->nodes, connectivity[nearest_node]->nodes[0]);
		nearestPt=annCopyPt(dim, dataPts[nearest_node]);
		cout<<"nearest point is at x: "<<nearestPt[0]<<" y: "<<nearestPt[1]<<endl;
		//next point
		next_way_point = connectivity[nearest_node]->nodes[idx];
		best_neighbor.push_back(next_way_point);
		next_way_point--;
	
		queryPt=annCopyPt(dim, dataPts[next_way_point]);
		cout<<"next point is "<<next_way_point<<" located at x: "<<queryPt[0]<<" y: "<<queryPt[1]<<endl;
		cout<<"distance to goal"<<J[next_way_point]<<endl;
		//calculating steering angle 
		car_state->gamma = atan2((queryPt[1] - currentPt[1]), (queryPt[0] - currentPt[0]));
		cout<<"arctan ="<<car_state->gamma<<endl;
		if(car_state->gamma < 0) car_state->gamma += (2*PI);
		cout<<"gamma = "<<car_state->gamma<<endl;
		error_an =  (car_state->heading -car_state->gamma);		
		if (abs(error_an) > PI){
			error_an = (-1)*sign_d(error_an)*(2*PI - abs(error_an));
		}
		cout<<"error_an = "<<error_an<<endl;
		//steering = (200/15)*(180/PI) * (error_an+(PI/2)) + 350;
	
		steering = 600 / (1+exp((-error_an)/0.2)) + 1250;

		cout<<"steering = "<<steering<<endl;
	
		//send angle to Serial
		if (abs(round(steering) - (car_state->InTemp)) >= 5){
			car_state->InTemp = round(steering);
			cmd.mode = 'f';				        
			cmd.speed = 120;   			
			cmd.steering = round(steering);
			pub.publish(cmd);
			ros::spinOnce();
			loop_rate.sleep();
			std::cout<<"data: speed" << cmd.speed <<", steering" <<cmd.steering<<"\n";
		}	
	}while(J[nearest_node] != 0 && ros::ok());
	 delete [] nnIdx;
 delete [] dists;
 delete kdTree;
 annClose();
 free(car_state);
 free(J);
 free(connectivity);
	close(sockfd);
	while(ros::ok()) {
		cmd.mode = 'f';
		cmd.speed = 0;
		cmd.steering = 1550;
		pub.publish(cmd);
	}
	
return 0;
}
  
