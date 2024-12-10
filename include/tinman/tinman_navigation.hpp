/**
 * @file tinman_navigation.hpp
 * @author Keyur Borad, FNU Koustubh, Swaraj Rao (kborad@umd.edu)
 * @brief This file contains the declaration of the RobotNavigation class, which is responsible for navigation-related tasks.
 * @version 0.1
 * @date 2024-12-10
 * 
 * @copyright Copyright (c) 2024
 * 
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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;  ///< Publisher for initial pose.
  geometry_msgs::msg::Pose current_pose_;  // To store the current position

 public:
  /**
   * @brief Constructor for the RobotNavigation class.
   */
  RobotNavigation();


  /**
   * @brief Sets the initial pose of the robot.
   * @param msg Initial pose message.
   */
  void set_initial_pose();

  /**
   * @brief Navigates the robot to a detected can.
   * @param distance Distance to the detected can.
   */
  void navigateToCan(float distance);
/**
 * @brief A methoid to move the robot to a specific position.
 * 
 * @param x double
 * @param y double
 */
  void moveToPosition(double x, double y);
/**
 * @brief Get the Current Position object
 * 
 * @return geometry_msgs::msg::Pose 
 */
  geometry_msgs::msg::Pose getCurrentPosition() const;

/**
 * @brief Callback function for the pose subscriber.
 * 
 * @param msg 
 */
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

};

#endif  // TINMAN_NAVIGATION_HPP