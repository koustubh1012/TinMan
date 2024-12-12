#include "rclcpp/rclcpp.hpp"
#include "tinman.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);  // Initialize ROS 2
  auto node =
      std::make_shared<TinMan>();  // Create an instance of the TinMan class
  rclcpp::spin(node);              // Keep the node alive and process callbacks
  rclcpp::shutdown();              // Shutdown ROS 2
  return 0;
}
