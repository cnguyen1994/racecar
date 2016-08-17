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
using namespace std;

typedef struct {
	char mode;
	uint32_t speed;
	uint32_t steering;
}COMMAND;
static int sent;
static int fd = -1;
static COMMAND *action;
unsigned char rx_buf[32];
unsigned char tx_buf[32];
void get_cmds(const racecar::CMD::ConstPtr& msg);
void toArray(unsigned char *numberArray, int number);
void serialize_command(unsigned char *tx_buf, COMMAND *action);

/* ROS class for subscribing and publishing to different node */
class Subscriber {
public:
	/* class constructor */
	Subscriber() 
	{
		/* declare publisher and subscriber */
		sub_ = n_.subscribe("command", 1000, &Subscriber::callBack, this);
	}
	void callBack(const racecar::CMD::ConstPtr& msg) 
	{
		int i;
		n_written = 0;
		n_read = 0;
		action->mode = msg->mode;
		action->speed = msg->speed;
		action->steering = msg->steering;
		ROS_INFO("Received commands!");
		ROS_INFO("mode = %c, speed = %d, steering = %d \n",action->mode, action->speed, action->steering);
		/* get command */

		/* stream command */
		memset(tx_buf, 0, sizeof(tx_buf));
		memset(rx_buf, 0, sizeof(rx_buf));
		printf("command: %s \n", tx_buf);
		serialize_command(tx_buf, action); 
		n_written = write(fd, tx_buf, 10);
		printf("written %d bytes to port %d \n", n_written, fd);
		if (n_written < 0) {
			perror("Write error");
			exit(-1);
		}
		/*n_read = read(fd, rx_buf, 10);
		printf("read %d bytes from arduino\n", n_read);
		printf("rx_buf[0] = %c \n", rx_buf[0]);*/
		usleep(20000);
		tcflush(fd, TCOFLUSH);
	}
private:
	/* class private variable */
	ros::NodeHandle n_; 
  	ros::Subscriber sub_;
	size_t n_written;
        size_t n_read;
};
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
  	struct termios toptions;
	memset(&toptions, 0, sizeof(toptions));
	action = (COMMAND *)calloc(sizeof(COMMAND),1);
	action->speed = 0;
	action->steering = 1500;
	action->mode = 'f';
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
	cfmakeraw(&toptions);

	cfsetispeed(&toptions, B115200);
	cfsetospeed(&toptions, B115200);
	/* 8 bits, no parity, one stop bits */
	//toptions.c_cflag |= (CS8 | CREAD | CLOCAL);
	//toptions.c_cflag &= ~(CSTOPB | CSIZE | PARENB);
	/* no hardware flow control */
	//toptions.c_cflag &= ~CRTSCTS;
	/* enable receriver, ignore status line */
	/* disable input/output flow control, disable restart chars */
	//toptions.c_iflag &= ~(IXON| IXANY | IXOFF);
	//toptions.c_iflag |= IGNPAR | IGNCR;
	//toptions.c_lflag |= ICANON;
	//toptions.c_oflag &= ~OPOST;
	
	/* disable canonical input, disable echo,
 	disable visually erase chars,disable terminal-generated signals */
	/* commit the serial port settings */
	if(tcsetattr(fd, TCSANOW, &toptions) != 0) {
		perror("Socket error");
		close(fd);
		return -1;
	}
	sleep(2);
	tcflush(fd, TCIOFLUSH);
	/* set up subscription */ 
	ros::init(argc, argv, "Serial");
	Subscriber sub;
	ros::Rate loop_rate(50);
	ros::Duration time = loop_rate.expectedCycleTime();
	std::cout <<time<<'\n';
	printf("start serial program \n");
	ROS_INFO("streaming command");
	//rx_buf[0] = '0';
	//ros::spin();
	do {
		//memset(tx_buf, 0, sizeof(rx_buf));
		ros::spinOnce();
		loop_rate.sleep();
	}while(ros::ok());

	
	free(action);
	close(fd);
	fd = -1;
	return 0;
}
