#include "ros/ros.h"
#include "std_msgs/String.h"
#include "racecar/LOC.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sstream>


typedef struct {
	float x;
	float y;
	float heading;
}LOCALIZATION;

static LOCALIZATION *loc;

char** parse_str(char *a_str, const char delimiter);
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
int main (int argc, char **argv) {
	/* init ros service */
	ros::init(argc, argv, "vicon");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<racecar::LOC>("localization", 1000);
	::racecar::LOC loc_data;
	/* init TCP/IP protocol */
	int sockfd, portno, num;
	struct sockaddr_in serv_addr;
	struct hostent *server;
	char buffer[256];
	/* communicate with vicon on port 30000 */
	portno = 30000;
	sleep(3); //wait for wifi to connect
	/* get host ip */
	server = gethostbyname("192.168.0.2");
	if (server == NULL)
		ROS_INFO("ERROR no such host");
	/* create socket */
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
		ROS_INFO("ERROR opening socket \n");
	/* setting up tcp/ip struct */
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, (char*)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(portno);
	/* connecting the host */
	if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
		ROS_INFO("ERROR connecting");
	ROS_INFO("Host connected");
	sleep(2);
	int count = 0;
	ros::Rate loop_rate(100);
	loc = (LOCALIZATION *)malloc(sizeof(LOCALIZATION));
	while(ros::ok()) {
		/* publishing at 10Mhz */
		//ros::Rate loop_rate(10);

		bzero(buffer, 256);
		strcpy(buffer, "get_state2minimal");
		num = write(sockfd, buffer, strlen(buffer));
		if (num < 0)
			ROS_INFO("ERROR writing to socket");
		bzero(buffer, 256);
		num = read(sockfd, buffer, 255);
		if (num < 0) {
			ROS_INFO("ERROR reading from socket");	
		}
		
		char **tokens = parse_str(buffer, ',');
		loc_data.x_cor = atof(*(tokens));
		loc_data.y_cor = atof(*(tokens + 1));
		loc_data.heading = atof(*(tokens +2));
		ROS_INFO("publishing: x: %f, y: %f, heading: %f", loc_data.x_cor, loc_data.y_cor, loc_data.heading);
		chatter_pub.publish(loc_data);
		ros::spinOnce();
		loop_rate.sleep();
		count++;

	}
	close(sockfd);
	sockfd =-1;
	return 0;
}
