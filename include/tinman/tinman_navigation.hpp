/**
 * @file robot_navigation.hpp
 * @brief Header file for the RobotNavigation class, handling navigation
 * functionalities.
 */

#ifndef TINMAN_NAVIGATION_HPP
#define TINMAN_NAVIGATION_HPP

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * @class RobotNavigation
 * @brief Class responsible for navigation-related tasks.
 */
class RobotNavigation : public rclcpp::Node {
 private:
  geometry_msgs::msg::Pose
      target_position;  ///< Target position for navigation.
  sensor_msgs::msg::LaserScan
      scan_data;  ///< Laser scan data for obstacle detection.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;  ///< Publisher for initial pose.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;
  
  geometry_msgs::msg::Pose current_pose_;  // To store the current position

 public:
  /**
   * @brief Constructor for the RobotNavigation class.
   */
  RobotNavigation();

  /**
   * @brief Generates a navigation goal.
   * @return The generated goal as a Pose.
   */
  geometry_msgs::msg::Pose generateGoal();

  /**
   * @brief Sets the initial pose of the robot.
   * @param msg Initial pose message.
   */
  void set_initial_pose();

  /**
   * @brief Sends the navigation goal to the robot.
   */
  void sendNavGoal();

  /**
   * @brief Cancels the current navigation goal.
   */
  void cancelNavGoal();

  /**
   * @brief Controls the robot's navigation behavior.
   */
  void tinmanController();

  /**
   * @brief Navigates the robot to a detected can.
   * @param distance Distance to the detected can.
   */
  void navigateToCan(float distance);

  void moveToPosition(double x, double y);

  geometry_msgs::msg::Pose getCurrentPosition() const;

  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // void poseCallback(const
  // geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};

#endif  // TINMAN_NAVIGATION_HPP