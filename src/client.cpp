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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;
static int vertex_size; 
static int sockfd = -1;

void error(const char *msg) {
  perror(msg);
  exit(0);
};

int main(int argc, char **argv) {
	FILE *fp;
	int portno, num;
	struct sockaddr_in serv_addr;
	struct hostent *server;
	char buffer[2056];

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

	fp = fopen("/home/ubuntu/catkin_ws/rrg_node.dat", "w");
	if(fp == NULL) {
		cout<<"ERROR: open file"<<endl;
		exit(-1);
	}
	sleep(2);

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

	for(int i = 0; i<vertex_size; i++) {
	  bzero(buffer, 2056);
	  strcpy(buffer, "get_graph");
	  num = write(sockfd, buffer, strlen(buffer));
	  if(num <0)error("ERROR: writing to socket");
	  bzero(buffer, 2056);
	  num = read(sockfd, buffer, 2055);
	  if(num<0)error("ERROR: reading from socket");
	  strcat(buffer, "\n");
	  fputs(buffer,fp);
	  cout<<"get graph "<<i<<endl;
	 }
	fclose(fp);
	fp = fopen("/home/ubuntu/catkin_ws/rrg_j.dat", "w");
	if(fp == NULL) {
		cout<<"ERROR: open file"<<endl;
		exit(-1);
	}
	for(int i = 0; i<vertex_size; i++) {
	  bzero(buffer, 2056);
	  strcpy(buffer, "get_j");
	  num = write(sockfd, buffer, strlen(buffer));
	  if(num <0)error("ERROR: writing to socket");
	  bzero(buffer, 2056);
	  num = read(sockfd, buffer, 2055);
	  if(num<0)error("ERROR: reading from socket");
	  strcat(buffer, "\n");
	  fputs(buffer,fp);
	cout<<"get dist "<<i<<endl;
	}

	fclose(fp);
	fp = fopen("/home/ubuntu/catkin_ws/rrg_connect.dat", "w");
	if(fp == NULL) {
		cout<<"ERROR: open file"<<endl;
		exit(-1);
	}
	int node =0;
	int size =0;
	for(int i = 0; i<vertex_size; i++) {
	  bzero(buffer, 2056);
	  strcpy(buffer, "get_size_connect");
	  num = write(sockfd, buffer, strlen(buffer));
	  if(num <0)error("ERROR: writing to socket");
	  bzero(buffer, 2056);
	  num = read(sockfd, buffer, 2055);
	  if(num<0)error("ERROR: reading from socket");
	  sscanf(buffer, "%d", &size);
	  std::stringstream ss;
	cout<<"get connect "<<i<<endl;
	  for(int j =0; j<size; j++) {
	   bzero(buffer, 2056);
	   strcpy(buffer, "get_connect");
	   num = write(sockfd, buffer, strlen(buffer));
	   if(num <0)error("ERROR: writing to socket");
	   bzero(buffer, 2056);
	   num = read(sockfd, buffer, 2055);
	   if(num<0)error("ERROR: reading from socket");
	   sscanf(buffer, "%d", &node);
	   ss<<node<<", ";
	  }
	  ss<<endl;
	  string ss_tmp = ss.str();
	  char *buff = const_cast<char*>(ss_tmp.c_str());
	  fputs(buff, fp);
	 }
	 fclose(fp);
	close(sockfd);
	return 0;
}
