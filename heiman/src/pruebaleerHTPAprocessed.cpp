#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "sensor_msgs/Image.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/times.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>


#include <sstream>

#include "../include/heiman/globals.h"

#include "heiman/temperatures.h"


int temperaturas[992];

//void HTPAoutputCallback(const std_msgs::String::ConstPtr& msg)
void HTPAoutputCallback(const heiman::temperatures::ConstPtr& msg)
{
ROS_INFO("leemos HTPAprocessed");
ROS_INFO("i = 0 %d", msg->temperaturas[0]);
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "pruebaleerHTPAprocessed");
	ros::NodeHandle n;
 
	ros::Subscriber sub = n.subscribe("HTPAprocessed", 0, HTPAoutputCallback);

	ros::Rate loop_rate(10);
int count = 0;
	while (ros::ok())
	  {	  
	    ros::spinOnce();
	    loop_rate.sleep();
	  }


	return 0;
}

