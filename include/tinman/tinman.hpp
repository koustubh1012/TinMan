/**
 * @file tinman.hpp
 * @brief Header file for the TinMan class, the main controller for the robot.
 */

#ifndef TINMAN_HPP
#define TINMAN_HPP

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "can_detection.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "tinman_navigation.hpp"
#include "manipulation.hpp"
#include <cmath>


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
    geometry_msgs::msg::Pose getCurrentPosition() const;
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void testNavigation(double x, double y);
};

#endif // TINMAN_HPP
