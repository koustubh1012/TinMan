/**
 * @file level2_integration_test.cpp
 * @brief This cpp file is a level 2 integration file to verify the unit test of publishing topic "cmd_vel".
 */

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "tinman.hpp"

using namespace std::chrono_literals;
using std_msgs::msg::String;

////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////
auto Logger = rclcpp::get_logger("");  // create an initial Logger

/**
 * @brief Class MyTestsFixture to make a test fixture
 */
class MyTestsFixture {
 public:
  /**
   * @brief Construct a new MyTestsFixture object
   */
  MyTestsFixture() {
    /**
     * 1.) Create the node that performs the test (aka Integration test node)
     */
    testerNode = rclcpp::Node::make_shared("Level2IntegrationTest");
    Logger = testerNode->get_logger();  // make sure messages will appear in rqt_console

    /**
     * 2.) Declare a parameter for the duration of the test
     */
    testerNode->declare_parameter<double>("test_duration");

    /**
     * 3.) Get the test duration value
     */
    TEST_DURATION = testerNode->get_parameter("test_duration").get_parameter_value().get<double>();
    RCLCPP_INFO_STREAM(Logger, "Got test_duration =" << TEST_DURATION);
  }

  ~MyTestsFixture() {}

 protected:
  double TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

////////////////////////////////////////////////
// Test Case: Verify cmd_vel topic behavior
////////////////////////////////////////////////

TEST_CASE_METHOD(MyTestsFixture, "test topic cmd_vel", "[topic]") {
  /**
   * Flag to indicate if a message was received and its content verified
   */
  bool message_received = false;

  /**
   * Subscribe to the cmd_vel topic
   */
  auto subscriber = testerNode->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [&message_received](const geometry_msgs::msg::Twist msg) {
        RCLCPP_INFO(Logger, "Received cmd_vel message: linear.x=%.2f, angular.z=%.2f",
                    msg.linear.x, msg.angular.z);
        
        // Validate the received message content
        if (msg.linear.x == 1.0 && msg.angular.z == 0.5) {
          message_received = true;
        }
      });

  /**
   * Create a publisher to send data to the cmd_vel topic
   */
  auto publisher = testerNode->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  /**
   * Publish a message to cmd_vel
   */
  auto publish_message = [&publisher]() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 1.0;  // Set linear velocity
    msg.angular.z = 0.5; // Set angular velocity
    RCLCPP_INFO(Logger, "Publishing cmd_vel: linear.x=1.0, angular.z=0.5");
    publisher->publish(msg);
  };

  /**
   * Perform the test by publishing and verifying the message
   */
  rclcpp::Rate rate(5.0); // 5Hz
  auto start_time = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - start_time;
  auto timeout = rclcpp::Duration::from_seconds(TEST_DURATION);
  RCLCPP_INFO(Logger, "Starting test: timeout=%.2f seconds", timeout.seconds());

  while (!message_received && (duration < timeout)) {
    publish_message(); // Publish a message to cmd_vel
    rclcpp::spin_some(testerNode); // Process callbacks
    rate.sleep();
    duration = rclcpp::Clock().now() - start_time;
  }

  RCLCPP_INFO(Logger, "Test finished: duration=%.2f seconds, message_received=%s",
              duration.seconds(), message_received ? "true" : "false");

  // Assert that the message was received and its content was valid
  CHECK(message_received);
}
