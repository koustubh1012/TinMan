#include "rclcpp/rclcpp.hpp"
#include "can_detection.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class CanDetectionTester : public rclcpp::Node {
public:
    CanDetectionTester() : Node("can_detection_tester") {
        // Create a subscriber to a camera topic (adjust the topic name accordingly)
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&CanDetectionTester::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "CanDetectionTester initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    CanDetection detector_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Process the image using CanDetection
        detector_.setCameraFrame(*msg);
        float centroid_x = detector_.trackCentroid();

        if (centroid_x >= 0) {
            RCLCPP_INFO(this->get_logger(), "Green can detected at centroid x: %.2f", centroid_x);
        } else {
            RCLCPP_WARN(this->get_logger(), "No green can detected.");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanDetectionTester>());
    rclcpp::shutdown();
    return 0;
}