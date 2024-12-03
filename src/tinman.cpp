#include "rclcpp/rclcpp.hpp"
#include "can_detection.hpp"
#include "tinman_navigation.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class TinManController : public rclcpp::Node {
public:
    TinManController() : Node("TinMan_Command") {
        // Create a subscriber to a camera topic (adjust the topic name accordingly)
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&TinManController::imageCallback, this, std::placeholders::_1));

        nav_obj.setInitialPose();

        RCLCPP_INFO(this->get_logger(), "TinManController initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    CanDetection detection_obj;
    RobotNavigation nav_obj;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Process the image using CanDetection
        detection_obj.setCameraFrame(*msg);
        float centroid_x = detection_obj.trackCentroid();

        // if (centroid_x >= 0) {
        //     RCLCPP_INFO(this->get_logger(), "Green can detected at centroid x: %.2f", centroid_x);
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "No green can detected.");
        // }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TinManController>());
    rclcpp::shutdown();
    return 0;
}
