/**
 * @file tinman.hpp
 * @author Keyur Borad, FNU Koustubh, Swaraj Rao (kborad@umd.edu)
 * @brief This file contains the declaration of the TinMan class, which is the main class for the project.
 * @version 0.1
 * @date 2024-12-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef TINMAN_HPP
#define TINMAN_HPP

#pragma once

#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "can_detection.hpp"
#include "manipulation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tinman_navigation.hpp"

class TinMan : public rclcpp::Node {
 public:
  TinMan();
  ~TinMan() = default;

 private:
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;

  // Helpers
  CanDetection detector_;
  std::shared_ptr<RobotNavigation> nav_obj_;
  std::shared_ptr<Manipulation> manipulator_obj_;

  // Pose and navigation
  geometry_msgs::msg::Pose current_pose_;
  double goal_x, goal_y;
  const double tolerance;
  bool nav_to_bin;

  // Methods
  /**
   * @brief Get the Current Position object
   * 
   * @return geometry_msgs::msg::Pose 
   */
  geometry_msgs::msg::Pose getCurrentPosition() const;
  /**
   * @brief Callback function for the pose subscription.
   * 
   * @param msg Pose message
   */
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  /**
   * @brief Callback function for the image subscription.
   * 
   * @param msg Image message
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  /**
   * @brief Method to navigate the robot to a specific position.
   * 
   * @param x double
   * @param y double
   */
  void testNavigation(double x, double y);
};

#endif  // TINMAN_HPP
