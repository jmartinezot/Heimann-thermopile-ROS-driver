#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/times.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <sstream>

#include "../include/heiman/globals.h"

int main(int argc, char **argv)
{

	bool HTPA_initialized;

	ros::init(argc, argv, "startHTPA");
	ros::NodeHandle n;

	ros::Rate loop_rate(10);
	
	ros::param::del("/HTPA_started");

	int sock; // the socket to communicate with the device
	//int port_number; // the port number to communicate with the device
	struct sockaddr_in HTPA_addr; // structure of the data on the client side
	struct sockaddr_in PC_addr; // structure of the data on the server side

	// creates the socket to communicate with the HTPA device
	sock = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	// first, HTPA_addr is filled with the right data
	memset( &HTPA_addr, 0, sizeof (HTPA_addr));
	HTPA_addr.sin_family = AF_INET;
	HTPA_addr.sin_port = htons (PORT_HTPA);
	HTPA_addr.sin_addr.s_addr = inet_addr (IP_HTPA);

	// then, PC_addr is filled with the right data
	memset( &PC_addr, 0, sizeof (PC_addr));
	PC_addr.sin_family = AF_INET;
	PC_addr.sin_port = htons (PORT_PC);
	PC_addr.sin_addr.s_addr = inet_addr (IP_PC);

	while (!ros::param::has("/HTPA_initialized"))
		{
			sleep(1);
			std::cout << "Waiting in startHTPA" << std::endl;
		}

	if(bind(sock, (struct sockaddr *)&PC_addr, sizeof(PC_addr)) == -1)
	{
	    perror("Server-bind() error lol! in startHTPA");
	//    exit(1);
	}

	int bytes_sent;
	char text = 'K';

	//sleep(10);

	ros::param::get("/HTPA_initialized", HTPA_initialized);
	
	if (HTPA_initialized)

	{
		bytes_sent = sendto (sock, &text, 1 , 0,
						 (struct sockaddr *)&HTPA_addr,
						  sizeof(HTPA_addr));

		ros::param::set("/HTPA_initialized", false); // esto probablemente sobre
		close(sock);
		ros::param::set("/HTPA_started", true);
	}

	return 0;
}

