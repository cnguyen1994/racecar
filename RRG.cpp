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
using namespace std;

typedef struct {
  float x;
  float y;
}GRAPH;


typedef struct {
  int *nodes;
}CONNECT;


static GRAPH **G;
static float *J;
static CONNECT **connectivity;
static int vertex_size = 0;
static int sockfd=-1;

void error(const char *msg) {
  perror(msg);
  exit(0);
};

int find_min(float *distance, int *elements, int size) {
  int small = distance[elements[1]];
  for(int i = 2; i<=size; i++) {
    if(distance[elements[i]] < small) small = distance[elements[i]];
  }
  return small;
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


int main(int argc, char **argv) {

  int portno, num;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  char buffer[2056];
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

  /*-ViCon client initialization end*/
  
  /*-Get graph-*/
  bzero(buffer, 2056);
  strcpy(buffer, "get_size");
  num = write(sockfd, buffer, strlen(buffer));
  if (num < 0)
       error("ERROR writing to socket");
  printf("- done sending reques\n");
  bzero(buffer, 2056);
  num = read(sockfd, buffer, 2056);
  if (num < 0)
 	error("ERROR reading from socket");  
  printf("-received data-");
  sscanf(buffer, "%d", &vertex_size);
  cout<<"the size of Graph is "<<vertex_size<<endl;
  G = (GRAPH **) malloc(sizeof(GRAPH*) * (vertex_size));
  J = (float *) calloc(sizeof(float*), (vertex_size));
  connectivity = (CONNECT**) malloc(sizeof(CONNECT*) * (vertex_size));
for(int i = 0; i<vertex_size; i++) {
  G[i] = (GRAPH *) calloc(sizeof(GRAPH),1);
  bzero(buffer, 2056);
  strcpy(buffer, "get_graph");
  num = write(sockfd, buffer, strlen(buffer));
  if(num <0)error("ERROR: writing to socket");
  bzero(buffer, 2056);
  num = read(sockfd, buffer, 2055);
  if(num<0)error("ERROR: reading from socket");
  char **tokens = parse_str(buffer, ',');
  G[i]->x = atof(*(tokens));
  G[i]->y = atof(*(tokens+1));
  cout<<"node "<<i<<" is: [x="<<G[i]->x<<", y="<<G[i]->y<<"]"<<endl;
 }

for(int i = 0; i<vertex_size; i++) {
  bzero(buffer, 2056);
  strcpy(buffer, "get_j");
  num = write(sockfd, buffer, strlen(buffer));
  if(num <0)error("ERROR: writing to socket");
  bzero(buffer, 2056);
  num = read(sockfd, buffer, 2055);
  if(num<0)error("ERROR: reading from socket");
  sscanf(buffer, "%f", &(J[i]));
  cout<<"node "<<i<<" distance to goal is: "<<J[i]<<endl;
 }

for(int i = 0; i<vertex_size; i++) {
  connectivity[i] = (CONNECT *) malloc(sizeof(CONNECT));
  bzero(buffer, 2056);
  strcpy(buffer, "get_size_connect");
  num = write(sockfd, buffer, strlen(buffer));
  if(num <0)error("ERROR: writing to socket");
  bzero(buffer, 2056);
  num = read(sockfd, buffer, 2055);
  if(num<0)error("ERROR: reading from socket");
  int size = 0;
  sscanf(buffer, "%d", &size);
  connectivity[i]->nodes = (int *) malloc(sizeof(int) * (size+1));
  connectivity[i]->nodes[0] = size;
 for(int j =1; j<=size; j++) {
   bzero(buffer, 2056);
  strcpy(buffer, "get_connect");
  num = write(sockfd, buffer, strlen(buffer));
  if(num <0)error("ERROR: writing to socket");
  bzero(buffer, 2056);
  num = read(sockfd, buffer, 2055);
  if(num<0)error("ERROR: reading from socket");
  sscanf(buffer, "%d", &(connectivity[i]->nodes[j]));
	 cout<<"node "<<i<<" connect to nodes "<<connectivity[i]->nodes[j]<<endl;
 }
 }
 list<int> trajectory;
 for(int i = 0; i<vertex_size; i++) {
   if(J[i] == 0) {
     int best = find_min(J, connectivity[i]->nodes, connectivity[i]->nodes[0]);
     trajectory.push_back(best);
   }
   else {
     trajectory.push_back(i);
   }
 }
 cout<<"best path found!"<<endl;
 list<int>::const_iterator iterator;
 for(iterator = trajectory.begin(); iterator != trajectory.end(); iterator ++){
   int best_node = *(iterator);
   cout<<"node "<<best_node<<"is in the best path"<<endl;
 }
  
 free(G);
 free(J);
 free(connectivity);
close(sockfd);
return 0;
}
  
