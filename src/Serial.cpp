#include "ros/ros.h"
#include "racecar/CMD.h"
#include "std_msgs/String.h"
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>


typedef struct {
	char mode;
	uint32_t speed;
	uint32_t steering;
}COMMAND;
static int fd = -1;
static size_t n_written = 0;
static COMMAND *action;
void get_cmds(const racecar::CMD::ConstPtr& msg);
void toArray(unsigned char *numberArray, int number);
void serialize_command(unsigned char *tx_buf, COMMAND *action);

void get_cmds(const racecar::CMD::ConstPtr& msg) {
	action->mode = msg->mode;
	action->speed = msg->speed;
	action->steering = msg->steering;
	ROS_INFO("Received commands!");
	ROS_INFO("mode = %c, speed = %d, steering = %d \n",action->mode, action->speed, action->steering);
}

void serialize_command(unsigned char *tx_buf, COMMAND *action)
{
	unsigned char *serial_ptr;
	unsigned char speed_buf[50] = {0};
	unsigned char steering_buf[50] = {0};
	toArray(speed_buf, action->speed);
	toArray(steering_buf, action->steering);
	tx_buf[0] = action->mode;
	serial_ptr = tx_buf+1;
	memcpy(serial_ptr, speed_buf, 4);
	serial_ptr += 4;
	memcpy(serial_ptr, steering_buf, 4);
	tx_buf[9] = '#';
	tx_buf[10] = '\0';
	
}
void toArray(unsigned char *numberArray, int number) {
	int n = 4;
	int i;
	for(i=n-1; i>=0; i--, number/=10) {
		numberArray[i] = (number % 10) + '0';
	}
}
int main(int argc, char **argv) {
	int count, i;
	unsigned char rx_buf[256];
	unsigned char tx_buf[256];
  	struct termios toptions;
	memset(&toptions, 0, sizeof(toptions));
	action = (COMMAND *)calloc(sizeof(COMMAND),1);
	/* open serial port */
	sleep(3); //wait for tty to init
	fd = open("/dev/arduino_uno", O_RDWR | O_NOCTTY);
	printf("fd opened as %d\n", fd);
	  
	/* wait for the Arduino to reboot */
	usleep(3500000);
	printf("done waiting\n");

	/* get current serial port settings */
	if(tcgetattr(fd, &toptions) != 0) {
		perror("Socket error");
		close(fd);
		return -1;
	}

	/* set 9600 baud both ways */
	cfsetispeed(&toptions, B115200);
	cfsetospeed(&toptions, B115200);

	/* 8 bits, no parity, no stop bits */
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag |= 0;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag = (toptions.c_cflag & ~CSIZE) | CS8;
	
	/* no hardware flow control */
	toptions.c_cflag &= ~CRTSCTS;
	/* enable receriver, ignore status line */
	toptions.c_cflag |= (CLOCAL | CREAD);
	/* disable input/output flow control, disable restart chars */
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
	/* disable canonical input, disable echo,
 	disable visually erase chars,disable terminal-generated signals */
	/* commit the serial port settings */
	if(tcsetattr(fd, TCSANOW, &toptions) != 0) {
		perror("Socket error");
		close(fd);
		return -1;
	}

	/* set up subscription */ 
	ros::init(argc, argv, "Serial");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("command", 1000, get_cmds);
	ros::Rate loop_rate(100);
	ros::Duration time = loop_rate.expectedCycleTime();
	std::cout <<time<<'\n';
	printf("start serial program \n");
	ROS_INFO("streaming command");
	while(ros::ok()) {
		/* stream command */
		serialize_command(tx_buf, action); 
		printf("command: %s \n", tx_buf);
		n_written = write(fd, tx_buf, 10);
		if (n_written < 0) {
			perror("Write error");
			return -1;
		}
		//usleep(50000);
		printf("written %d bytes to port %d \n", n_written, fd);
		memset(tx_buf, 0, sizeof(tx_buf));
		ros::spinOnce();
		loop_rate.sleep();
	}
	free(action);
	close(fd);
	fd = -1;
	return 0;
}