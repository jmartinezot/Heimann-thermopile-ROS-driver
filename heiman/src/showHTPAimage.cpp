#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
// Eli
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
namespace enc = sensor_msgs::image_encodings;
//------
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Eli
  //  sensor_msgs::cv_bridge bridge;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    //cvShowImage("view", bridge.imgMsgToCv(msg, "bgr8"));
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", e.what());
    return;
  }
  cv::imshow("view", cv_ptr->image);
  //------

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "showHTPAimage");
  ros::NodeHandle nh;
  cvNamedWindow("view");
  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("HTPAimage", 1, imageCallback);

  ros::spin();
  cvDestroyWindow("view");
}

