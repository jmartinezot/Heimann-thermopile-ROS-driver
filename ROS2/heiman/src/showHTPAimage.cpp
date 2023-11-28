#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageDisplayNode : public rclcpp::Node {
public:
    ImageDisplayNode() : Node("image_display_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/HTPAimage", 10,
            std::bind(&ImageDisplayNode::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::imshow("HTPA Image", cv_ptr->image);
            cv::waitKey(1);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to display image: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageDisplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
