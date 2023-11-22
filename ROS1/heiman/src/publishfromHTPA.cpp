#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt8.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/times.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <sstream>

#include <heiman/globals.h>

int 
main(int argc, char **argv)
{
  ros::init(argc, argv, "publishfromHTPA");
  ros::NodeHandle n;
  //ros::Publisher HTPAoutput_pub = n.advertise<std_msgs::String>("HTPAoutput", 1000);
  ros::Publisher HTPAoutput_pub = n.advertise<std_msgs::UInt8MultiArray>("HTPAoutput", 0);
  
  ros::Rate loop_rate(20);
  
  int sock; // the socket to communicate with the device
  //int port_number; // the port number to communicate with the device
  struct sockaddr_in PC_addr; // structure of the data on the client side
  
  // creates the socket to communicate with the HTPA device
  sock = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  
  // first, PC_addr is filled with the right data
  memset( &PC_addr, 0, sizeof (PC_addr));
  PC_addr.sin_family = AF_INET;
  PC_addr.sin_port = htons (PORT_HTPA);
  PC_addr.sin_addr.s_addr = inet_addr (IP_PC);
  
  	while (!ros::param::has("/HTPA_started"))
		{
			sleep(1);
			std::cout << "Waiting in publishfromHTPA" << std::endl;
		}
  
  if(bind(sock, (struct sockaddr *)&PC_addr, sizeof(PC_addr)) == -1)
    {
      perror("Server-bind() error lol! in publishfromHTPA");
      exit(1);
    }
  
  while (ros::ok())
    {
      //std_msgs::String msg;
      std_msgs::UInt8MultiArray msg;
      //std::stringstream ss;
      
      char* stbis=(char*)malloc(MTU*sizeof(char));
      int iRc;
      //ROS_INFO("PublishFromHTPA BEFORE recvfrom :Received iRc %d from socket", iRc);
      iRc = recvfrom(sock,(char*)stbis,MTU*sizeof(char),0,NULL,NULL);
      //ROS_INFO("PublishFromHTPA:Received iRc %d from socket", iRc);
      if (iRc > 0)
	{
	  //ROS_INFO("iRc = %d",iRc);
	  //msg.data = (uint8*)stbis; 
	  
	  // Eli
	  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());	
	  //msg.set_data_size(iRc);
	  //msg.layout.set_dim_size(1);
	  // ----------------
	  msg.layout.dim[0].size = iRc;
	  msg.layout.dim[0].stride = iRc;

	  msg.data.resize(iRc);
	  for (int ii = 0; ii < iRc; ii++)
	     msg.data[ii] = (unsigned char)stbis[ii];


	  //msg.data = (unsigned char*)stbis; 
	  //msg.data.append(stbis,iRc);
	  //ss << stbis;
	  //msg.data = ss.str();
	  
	  HTPAoutput_pub.publish(msg);
	}
      
      ros::spinOnce();
      
      loop_rate.sleep();
      free(stbis);
    }
  
  
  return 0;
}

