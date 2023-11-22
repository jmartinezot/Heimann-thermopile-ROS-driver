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
  ros::init(argc, argv, "initializeHTPA");
  ros::NodeHandle n;
  
  ros::Rate loop_rate(10);
  
  int sock; // the socket to communicate with the device
  //int port_number; // the port number to communicate with the device
  struct sockaddr_in HTPA_addr; // structure of the data on the server side
  
  ros::param::del("/HTPA_initialized");
  
  // creates the socket to communicate with the HTPA device
  sock = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  
  // first, HTPA_addr is filled with the right data
  memset( &HTPA_addr, 0, sizeof (HTPA_addr));
  HTPA_addr.sin_family = AF_INET;
  HTPA_addr.sin_port = htons (PORT_HTPA);
  HTPA_addr.sin_addr.s_addr = inet_addr (IP_HTPA);
  
  bind(sock, (struct sockaddr *)&HTPA_addr, sizeof(HTPA_addr));
  
  bool connected = false;
  
  while (!connected)
  
  {
	//bind(sock, (struct sockaddr *)&HTPA_addr, sizeof(HTPA_addr));

	int bytes_sent;
	char text[50];

	strcpy(text,"Bind HTPA series device");

	bytes_sent = sendto (sock, text, strlen(text) , 0,
			   (struct sockaddr *)&HTPA_addr,
			   sizeof(HTPA_addr));
	if (bytes_sent != -1) connected = true;
	std::cout << "bytes_sent: " << bytes_sent << std::endl;
	close(sock);
   }
  
  ros::param::set("/HTPA_initialized", true);
  
  return 0;
  
  //if(bind(sock, (struct sockaddr *)&HTPA_addr, sizeof(HTPA_addr)) == -1)
    //{
	//std::cout << "Wrong??" << std::endl;
      ////    perror("Server-bind() error lol!");
      ////    exit(1);
    //}
  
  //int bytes_sent;
  //char text[50];
  
  //strcpy(text,"Bind HTPA series device");
  
  //bytes_sent = sendto (sock, text, strlen(text) , 0,
		       //(struct sockaddr *)&HTPA_addr,
		       //sizeof(HTPA_addr));
  
  //ros::param::set("/HTPA_initialized", true);
  
  //return 0;
}

