/**
 * @file robot_navigation.hpp
 * @brief Header file for the RobotNavigation class, handling navigation functionalities.
 */

#ifndef TINMAN_NAVIGATION_HPP
#define TINMAN_NAVIGATION_HPP

#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * @class RobotNavigation
 * @brief Class responsible for navigation-related tasks.
 */
class RobotNavigation {
private:
    geometry_msgs::msg::Pose target_position; ///< Target position for navigation.
    sensor_msgs::msg::LaserScan scan_data; ///< Laser scan data for obstacle detection.
    int cans_collected; ///< Number of cans collected during navigation.

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
};

#endif // TINMAN_NAVIGATION_HPP