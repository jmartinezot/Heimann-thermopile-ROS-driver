#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
 
CvCapture* pCapture = NULL;

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  //cv::WImageBuffer3_b image( cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) );
  //sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");
 
//while (!(pCapture = cvCaptureFromAVI(argv[2])));

//pCapture = cvCaptureFromAVI(argv[2]); 
pCapture = cvCaptureFromCAM(0); 

if (pCapture == NULL)
ROS_INFO("pCapture es nulo");

  ros::Rate loop_rate(5);

//pub.publish(msg);
IplImage *source;
IplImage *destination;

  while (nh.ok()) {

source = cvQueryFrame( pCapture );
int source_width = source->width;
int source_height = source->height;
if (argc == 3) // me estan diciendo el tamanyo que queremos que tenga la imagen de salida
destination = cvCreateImage
( cvSize (atoi(argv[1]),atoi(argv[2])),
 source->depth, source->nChannels );
else
destination = cvCreateImage
( cvSize ((int)((source_width)/1),(int)((source_height)/1) ),
 source->depth, source->nChannels );

//use cvResize to resize source to a destination image
cvResize(source, destination);


//cv::WImageBuffer3_b image( cvQueryFrame( pCapture ));
//sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");

 sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(destination, "bgr8");

   pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}


