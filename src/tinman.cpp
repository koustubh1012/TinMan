#include "tinman.hpp"

TinMan::TinMan() : Node("tinman") {
    cans_collected = 0;
    cans_trashed = 0;
    nav_obj = RobotNavigation();
    detection_obj = CanDetection();
    manipulator_obj = Manipulation();

    auto image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image", 10, std::bind(&TinMan::imageCallback, this, std::placeholders::_1));

    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>( "/initialpose", 10);

    RCLCPP_INFO(this->get_logger(), "TinMan controller initialized.");
}

void TinMan::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    detection_obj.setCameraFrame(*msg);
    float centroid_x = detection_obj.trackCentroid();

    if (centroid_x >= 0) {
        RCLCPP_INFO(this->get_logger(), "Green can detected at centroid x: %.2f", centroid_x);
    } else {
        RCLCPP_WARN(this->get_logger(), "No green can detected.");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TinMan>());
    rclcpp::shutdown();
    return 0;
}
