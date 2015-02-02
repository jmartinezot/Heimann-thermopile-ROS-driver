#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt8.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/times.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <sstream>

#include "../include/heiman/globals.h"
#include "heiman/temperatures.h"

int main(int argc, char **argv)
{
	int temperaturas[992];
	ros::init(argc, argv, "publishHTPAprocessedfromfile");
	ros::NodeHandle n;

	ros::Publisher HTPAprocessed_pub = n.advertise<heiman::temperatures>("HTPAprocessed", 0);

	ros::Rate loop_rate(20);

	FILE* ficheroHTPA;
	ficheroHTPA = fopen(argv[1],"r");
	//ROS_INFO("dfsd");
	for (int i = 0; i < 992; i++)
		{
			fscanf(ficheroHTPA,"%d",&temperaturas[i]);
		}
	fclose(ficheroHTPA);

	while (ros::ok())
	  {
	    heiman::temperatures msg_processed;
	    for (int i = 0; i < 992; i++)
			msg_processed.temperaturas[i] = temperaturas[i];
		HTPAprocessed_pub.publish(msg_processed);
	    ros::spinOnce();
	    loop_rate.sleep();
	  }

	return 0;
}

