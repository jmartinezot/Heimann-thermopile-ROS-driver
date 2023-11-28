#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <sstream>

IplImage *HTPAimage;
std::atomic<bool> is_new_data_ready(false);
int zoom;
int lower_limit;
int upper_limit;

void convertTtoRGB(int T, int *R, int *G, int *B, int low_limit, int up_limit)
// convert a tempeature in Kelvin*10 into a RGB value
// the more blue the colder, the more red the hotter; lower_limit_kelvin_decimals = 2980 (273K + 25); 
// upper_limit_kelvin_decimals = 3180 (273K + 45) 
	{
		int lower_limit_kelvin_decimals = (low_limit + 273) * 10;
		int upper_limit_kelvin_decimals = (up_limit + 273) * 10;
		if (T < lower_limit_kelvin_decimals)
			{ *R = 0; *G = 0; *B = 0; return;}
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

void setPixelColor(int pixel, int R, int G, int B) {
    int pixel_i = pixel / 32;
    int pixel_j = pixel % 32;
    for (int zoom_i = 0; zoom_i < zoom; zoom_i++)
        for (int zoom_j = 0; zoom_j < zoom; zoom_j++) {
            uchar* imageData = (uchar*)(HTPAimage->imageData + (pixel_i * zoom + zoom_i) * HTPAimage->widthStep);
            imageData[(pixel_j * zoom + zoom_j) * HTPAimage->nChannels + 0] = B;
            imageData[(pixel_j * zoom + zoom_j) * HTPAimage->nChannels + 1] = G;
            imageData[(pixel_j * zoom + zoom_j) * HTPAimage->nChannels + 2] = R;
        }
}

void HTPAoutputCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    int length = msg->layout.dim[0].stride;

    int start = 0;
    int end = (length == 1058) ? 529 : 463;
    int displacement = (length == 1058) ? 0 : 529;

    for (int i = start; i < end; i++) {
        int temp = msg->data[i * 2] + (msg->data[i * 2 + 1]) * 256;
        int R, G, B;
        convertTtoRGB(temp, &R, &G, &B, lower_limit, upper_limit);
        
        int quotient32 = (i + displacement) / 32;
        int remainder32 = (i + displacement) % 32;
        bool odd = remainder32 % 2 == 1;
        int pixel = quotient32 * 32 + remainder32 / 2;
        if (odd) {
            pixel += 16;
        }

        setPixelColor(pixel, R, G, B);
    }
    is_new_data_ready = true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("convertimagefromHTPApublished");

    node->get_parameter("zoom", zoom);
    node->get_parameter("lower_limit", lower_limit);
    node->get_parameter("upper_limit", upper_limit);

    HTPAimage = cvCreateImage(cvSize(32 * zoom, 31 * zoom), IPL_DEPTH_8U, 3);

    image_transport::ImageTransport it(node);
    auto HTPAimage_pub = it.advertise("HTPAimage", 1);

    auto sub = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
    "HTPAoutput", rclcpp::QoS(10), HTPAoutputCallback);

    rclcpp::Rate loop_rate(20);  // Increase if necessary
    while (rclcpp::ok()) {
        if (is_new_data_ready) {
            // Publish the image
            cv_bridge::CvImage cv_image;
            cv_image.header.stamp = node->now();
            cv_image.header.frame_id = "image";
            cv_image.encoding = sensor_msgs::image_encodings::BGR8;
            cv_image.image = cv::cvarrToMat(HTPAimage);
            HTPAimage_pub.publish(cv_image.toImageMsg());

            // Reset the flag
            is_new_data_ready = false;
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    cvReleaseImage(&HTPAimage);
    rclcpp::shutdown();
    return 0;
}
