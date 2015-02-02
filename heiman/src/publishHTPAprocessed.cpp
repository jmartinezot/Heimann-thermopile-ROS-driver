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
void HTPAoutputCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
	int i,length;
ROS_INFO("HTPAoutputCallback de publishHTPAprocessed");
        length = msg->layout.dim[0].stride;
	// each set of 2 bytes is a temperature in Kelvin*10; first the low-byte, then the high-byte
	if (length == 1058)
	//if (first == true)
	// the first 529 bytes (0 .. 528)
	{
		for (i = 0; i < 529; i++)
		{
			// analyze every two bytes (i*2 and i*2+1)
			//int temp = (unsigned char)str[i*2] + ((unsigned char)str[i*2+1])*256;
			int temp = msg->data[i*2] + (msg->data[i*2+1])*256;
			temperaturas[i] = temp;
		}
	}
	else
	// the last 463 bytes (529 .. 991)
	{
		for (i = 0; i < 463; i++)
		{
			int temp = msg->data[i*2] + (msg->data[i*2+1])*256;
			temperaturas[i+529] = temp;
		}
		ROS_INFO("Vamos a publicar");
		//HTPAprocessed_pub.publish(msg_processed);
	}
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "publishHTPAprocessed");
	ros::NodeHandle n;
 
	ros::Publisher HTPAprocessed_pub = n.advertise<heiman::temperatures>("HTPAprocessed", 1);
	ros::Subscriber sub = n.subscribe("HTPAoutput", 0, HTPAoutputCallback);

	ros::Rate loop_rate(10);
int count = 0;
	while (ros::ok())
	  {
		  count++;
	    heiman::temperatures msg_processed;
	    for (int i = 0; i < 992; i++)
			msg_processed.temperaturas[i] = temperaturas[i];
		//ROS_INFO("Antes");
		//if (count % 2 == 0)
			HTPAprocessed_pub.publish(msg_processed);	
	    //ROS_INFO("Despues");	  
	    ros::spinOnce();
	    loop_rate.sleep();
	  }


	return 0;
}

