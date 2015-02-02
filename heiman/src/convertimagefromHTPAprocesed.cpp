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
// Eli
//#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include  <sensor_msgs/image_encodings.h>
// ---
#include <sstream>

#include "../include/heiman/globals.h"

//#define ZOOM 20

int zoom;

IplImage *HTPAimage;
bool first = true;
int count_loops;
int lower_limit, upper_limit;

void convertTtoRGB(int T, int *R, int *G, int *B, int low_limit, int up_limit)
// convert a tempeature in Kelvin*10 into a RGB value
// the more blue the colder, the more red the hotter; lower_limit_kelvin_decimals = 2980 (273K + 25); 
// upper_limit_kelvin_decimals = 3180 (273K + 45) 
	{
		int lower_limit_kelvin_decimals = (low_limit + 273) * 10;
		int upper_limit_kelvin_decimals = (up_limit + 273) * 10;
		if (T < lower_limit_kelvin_decimals)
			{ *R = 0; *G = 0; *B = 0; return;}
		//if (T > upper_limit_kelvin_decimals)
			//{ *R = 255; *G = 0; *B = 0; return;}
		if (T > upper_limit_kelvin_decimals)
			{ *R = 255; *G = 255; *B = 255; return;}
		double width = (upper_limit_kelvin_decimals - lower_limit_kelvin_decimals) / 2.0;
		double temperate_zone = (upper_limit_kelvin_decimals + lower_limit_kelvin_decimals) / 2.0;
		if (T < temperate_zone)
		{
			double percentage_temperate = (double)(T - lower_limit_kelvin_decimals) / (double)width;
			double percentage_cold = 1.0 - percentage_temperate;
			{
				*R = 0;
				*G = (int)(percentage_temperate * 255);
				*B = (int)(percentage_cold * 255);
			}	
			return;			
		}
		double percentage_hot = (double)(T - temperate_zone) / (double)width;
		double percentage_temperate = 1.0 - percentage_hot;
		{
			*R = (int)(percentage_hot * 255);
			*G = (int)(percentage_temperate * 255);
			*B = 0;
		}	
		return;
	}

//void HTPAoutputCallback(const std_msgs::String::ConstPtr& msg)
void HTPAoutputCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
	int i,length;
	std::stringstream ss;	

	count_loops = 0;
	//char str[MTU];

	// convert the binary string published in a ROS image

	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	//ss << msg->data.c_str();
	// we have to check if it is the first part of the image (1058 bytes) or the second one (1054 bytes)
	//sprintf(str,"%s",msg->data.c_str());
	//length = strlen(str);
        length = msg->layout.dim[0].stride;
	//printf("Length = %d\n",length);
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
			//printf("first_byte: %d second_byte: %d temp: %d \n",(unsigned char)str[i*2], (unsigned char)str[i*2+1],temp);
			int R, G, B;
			int quotient32 = i / 32;
			int remainder32 = i % 32;
			bool odd = remainder32 % 2 == 1;
			convertTtoRGB(temp, &R, &G, &B, lower_limit, upper_limit);
			int pixel = quotient32 * 32 + remainder32 / 2;	
			if (odd)
			{
				pixel += 16;
			}
			int pixel_i = pixel / 32;
			int pixel_j = pixel % 32;
			// this would be if only one pixel is written			
			/*((uchar *)(HTPAimage->imageData + pixel_i*HTPAimage->widthStep))[pixel_j*HTPAimage->nChannels + 0] = B;			
			((uchar *)(HTPAimage->imageData + pixel_i*HTPAimage->widthStep))[pixel_j*HTPAimage->nChannels + 1] = G;			
			((uchar *)(HTPAimage->imageData + pixel_i*HTPAimage->widthStep))[pixel_j*HTPAimage->nChannels + 2] = R;	
			*/
			for (int zoom_i = 0; zoom_i < zoom; zoom_i++)
			 for (int zoom_j = 0; zoom_j < zoom; zoom_j++)
			  {
	((uchar *)(HTPAimage->imageData + (pixel_i*zoom+zoom_i)*HTPAimage->widthStep))[(pixel_j*zoom+zoom_j)*HTPAimage->nChannels + 0] = B;			
	((uchar *)(HTPAimage->imageData + (pixel_i*zoom+zoom_i)*HTPAimage->widthStep))[(pixel_j*zoom+zoom_j)*HTPAimage->nChannels + 1] = G;			
	((uchar *)(HTPAimage->imageData + (pixel_i*zoom+zoom_i)*HTPAimage->widthStep))[(pixel_j*zoom+zoom_j)*HTPAimage->nChannels + 2] = R;	
		          }						
		}
	first = false;
	}
	//if (length == 99926)
	else
	// the last 463 bytes (529 .. 991)
	{
		for (i = 0; i < 463; i++)
		{
			// analyze every two bytes (i*2 and i*2+1)
			//int temp = (unsigned char)str[i*2] + ((unsigned char)str[i*2+1])*256;
                        int temp = msg->data[i*2] + (msg->data[i*2+1])*256;
			int R, G, B;
			int quotient32 = (i + 529)/ 32;
			int remainder32 = (i + 529) % 32;
			bool odd = remainder32 % 2 == 1;
			convertTtoRGB(temp, &R, &G, &B, lower_limit, upper_limit);
			int pixel = quotient32 * 32 + remainder32 / 2;	
			if (odd)
			{
				pixel += 16;
			}
			//pixel += 529; // this is the second part, the first 529 bytes have been already written down
			//printf("Pixel: %d\n", pixel);
			int pixel_i = pixel / 32;
			int pixel_j = pixel % 32;	
			/*		
			((uchar *)(HTPAimage->imageData + pixel_i*HTPAimage->widthStep))[pixel_j*HTPAimage->nChannels + 0] = B;			
			((uchar *)(HTPAimage->imageData + pixel_i*HTPAimage->widthStep))[pixel_j*HTPAimage->nChannels + 1] = G;			
			((uchar *)(HTPAimage->imageData + pixel_i*HTPAimage->widthStep))[pixel_j*HTPAimage->nChannels + 2] = R;	
			*/		
			for (int zoom_i = 0; zoom_i < zoom; zoom_i++)
			 for (int zoom_j = 0; zoom_j < zoom; zoom_j++)
			  {
	((uchar *)(HTPAimage->imageData + (pixel_i*zoom+zoom_i)*HTPAimage->widthStep))[(pixel_j*zoom+zoom_j)*HTPAimage->nChannels + 0] = B;			
	((uchar *)(HTPAimage->imageData + (pixel_i*zoom+zoom_i)*HTPAimage->widthStep))[(pixel_j*zoom+zoom_j)*HTPAimage->nChannels + 1] = G;			
	((uchar *)(HTPAimage->imageData + (pixel_i*zoom+zoom_i)*HTPAimage->widthStep))[(pixel_j*zoom+zoom_j)*HTPAimage->nChannels + 2] = R;	
		          }						
		}
	first = true;
	}
}


int main(int argc, char **argv)
{
	count_loops = 0;	

	ros::init(argc, argv, "convertimagefromHTPAprocessed");
	ros::NodeHandle n;
	
	ros::param::set("zoom", 20);
	lower_limit = 25;	
 	upper_limit = 45;
 	
	zoom = 20;
	
	HTPAimage = cvCreateImage(cvSize(32*zoom,31*zoom),IPL_DEPTH_8U,3);	
 
 	image_transport::ImageTransport it(n);
  	image_transport::Publisher HTPAimage_pub = it.advertise("HTPAimage", 1);
 
        //ros::Subscriber sub = n.subscribe("HTPAoutput", 1000, HTPAoutputCallback);
        ros::Subscriber sub = n.subscribe("HTPAoutput", 0, HTPAoutputCallback);

	ros::Rate loop_rate(10);

	while (ros::ok())
	  {
			count_loops++;
			if (count_loops == 100) 
			{
				ROS_WARN("Ten seconds without new HTPA output published");
				count_loops = 0;
			}
			
		int zoom_aux;	
		ros::param::get("zoom", zoom_aux);
		if (ros::param::has("lower_limit"))
			ros::param::get("lower_limit", lower_limit);		
		if (ros::param::has("upper_limit"))
			ros::param::get("upper_limit", upper_limit);		
		
 		if (zoom_aux != zoom)
		{
			zoom = zoom_aux;
			cvReleaseImage(&HTPAimage);
			HTPAimage = cvCreateImage(cvSize(32*zoom,31*zoom),IPL_DEPTH_8U,3);	 		
		}

	    // Eli
		cv_bridge::CvImagePtr cpt(new cv_bridge::CvImage);
		ros::Time time = ros::Time::now();
		cpt->header.stamp = time;
		cpt->header.frame_id = "image";
		cpt->encoding = "bgr8";
		cpt->image = HTPAimage; //HTPAimage;


		HTPAimage_pub.publish(cpt->toImageMsg());
	    //sensor_msgs::ImagePtr msg = sensor_msgs::cv_bridge::cvToImgMsg(HTPAimage, "bgr8");
	    //HTPAimage_pub.publish(msg);
	    //------
	    ros::spinOnce();

	    loop_rate.sleep();
	  }


	return 0;
}

