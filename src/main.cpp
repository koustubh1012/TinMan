#include "can_detection.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a shared pointer to the CanDetection node
    auto can_detection_node = std::make_shared<CanDetection>();

    // Spin the node to process callbacks
    rclcpp::spin(can_detection_node);

    // Shut down the ROS 2 client library
    rclcpp::shutdown();

    return 0;
}
