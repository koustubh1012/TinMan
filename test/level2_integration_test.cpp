/**
 * @file level2_integration_test.cpp
 * @brief This cpp file is a level2 integration file to verify the unit test of publishing topic "cmd_vel"
 */

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;

////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////
auto Logger = rclcpp::get_logger("");  // Create an initial Logger

/**
 * @brief Class MyTestsFixture to create a test fixture
 */
class MyTestsFixture {
 public:
  /**
   * @brief Construct a new MyTestsFixture object
   */
  MyTestsFixture() {

    testerNode = rclcpp::Node::make_shared("Level2IntegrationTest");
    Logger = testerNode->get_logger();  // Ensure messages will appear in rqt_console

    testerNode->declare_parameter<double>("test_duration");

    TEST_DURATION = testerNode->get_parameter("test_duration")
                        .get_parameter_value()
                        .get<double>();
    RCLCPP_INFO_STREAM(Logger, "Got test_duration = " << TEST_DURATION);

    rclcpp::sleep_for(3s);  // Sleep for 1 second

    subscriber = testerNode->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist &msg) {
          RCLCPP_INFO(Logger, "INSIDE CALLBACK");
        });

    std::string topic_name = "/cmd_vel";
    size_t publisher_count = testerNode->count_publishers(topic_name);

    if (publisher_count > 0) {
      RCLCPP_INFO_STREAM(Logger, "Found publisher for topic " << topic_name);
      got_topic = true;
    }
    else {
      RCLCPP_INFO_STREAM(Logger, "No publisher found for topic " << topic_name);
    }
  }

  ~MyTestsFixture() {}

 protected:
  double TEST_DURATION;
  bool got_topic = false;  // Flag to track if a valid message was received
  rclcpp::Node::SharedPtr testerNode;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
};

////////////////////////////////////////////////
// Test Case 1
////////////////////////////////////////////////

TEST_CASE_METHOD(MyTestsFixture, "test topic cmd_vel", "[topic]") {

  rclcpp::Rate rate(5.0);  // 5Hz checks
  auto start_time = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - start_time;
  auto timeout = rclcpp::Duration::from_seconds(TEST_DURATION);

  RCLCPP_INFO_STREAM(Logger, "Starting test: duration = " << duration.seconds()
                                                          << " timeout=" << timeout.seconds());

  while (!got_topic && (duration < timeout)) {
    rclcpp::spin_some(testerNode);  // Process callbacks
    rate.sleep();  // Sleep for the defined rate
    duration = rclcpp::Clock().now() - start_time;
  }

  RCLCPP_INFO_STREAM(Logger, "Test finished: duration = " << duration.seconds()
                                                          << ", got_topic = " << std::boolalpha
                                                          << got_topic);

  // Assert that a valid topic is published
  CHECK(got_topic);
}
