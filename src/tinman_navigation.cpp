#include "tinman_navigation.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

RobotNavigation::RobotNavigation() : Node("robot_navigation") {
  // Create a publisher for velocity commands
  velocity_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // Create a publisher for initial pose
  initial_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", 10);
  // Calling the set_initial_pose method
  set_initial_pose();
}
// Metjod to get the current position of the robot
geometry_msgs::msg::Pose RobotNavigation::getCurrentPosition() const {
  return current_pose_;
}

// Method to navigate the robot to a detected can
void RobotNavigation::navigateToCan(float centroid_x) {
  auto vel = geometry_msgs::msg::Twist();
  if (centroid_x == -2.0) {
    // Stop the robot
    vel.angular.z = 0.0;
    vel.linear.x = 0.0;
    RCLCPP_INFO(this->get_logger(),
                "Stopping robot due to large contour area.");
  } else {
    int center = 960;   // Center of the image
    int deadband = 50;  // Deadband around the center
    float scale_factor = 0.001;

    if (centroid_x < center - deadband) {
      vel.angular.z = scale_factor * (center - centroid_x);
      vel.linear.x = 0.0;
    } else if (centroid_x > center + deadband) {
      vel.angular.z = -scale_factor * (centroid_x - center);
      vel.linear.x = 0.0;
    } else {
      vel.angular.z = 0.0;
      vel.linear.x = 0.2;
    }
  }

  velocity_publisher_->publish(vel);
}

// Method to move the robot to a specific position
void RobotNavigation::moveToPosition(double x, double y) {
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  // Create an action client for /navigate_to_pose
  auto action_client = rclcpp_action::create_client<NavigateToPose>(
      shared_from_this(), "navigate_to_pose");

  // Wait for the action server to be available
  if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server /navigate_to_pose not available.");
    return;
  }

  // Create a goal message
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";                  // Use the map frame
  goal_msg.pose.header.stamp = this->get_clock()->now();  // Current time
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.position.z = 0.0;
  goal_msg.pose.pose.orientation.x = -0.0014;
  goal_msg.pose.pose.orientation.y = 0.0030;
  goal_msg.pose.pose.orientation.z = 0.4452;
  goal_msg.pose.pose.orientation.w = 0.8954;

  // Send the goal and wait for result
  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult
             &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(rclcpp::get_logger("RobotNavigation"),
                      "Goal reached successfully.");
        } else {
          RCLCPP_WARN(rclcpp::get_logger("RobotNavigation"),
                      "Failed to reach the goal.");
        }
      };

  action_client->async_send_goal(goal_msg, send_goal_options);
}

// Callback function for the pose subscriber
void RobotNavigation::poseCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose_ = msg->pose.pose;
  RCLCPP_INFO(this->get_logger(), "Current position updated: x=%.2f, y=%.2f",
              current_pose_.position.x, current_pose_.position.y);
}

// Method to set the initial pose of the robot
void RobotNavigation::set_initial_pose() {
  auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
  message.header.frame_id = "map";
  message.pose.pose.position.x = -4.0;
  message.pose.pose.position.y = -1.0;
  message.pose.pose.orientation.x = 0.0;
  message.pose.pose.orientation.y = 0.0;
  message.pose.pose.orientation.z = 0.0;
  message.pose.pose.orientation.w = 1.0;
  initial_pose_pub_->publish(message);
  RCLCPP_INFO(this->get_logger(), "INITAL POSE SET");
}
