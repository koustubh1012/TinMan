/**
 * @file tinman.hpp
 * @brief Header file for the TinMan class, the main controller for the robot.
 */

#ifndef TINMAN_HPP
#define TINMAN_HPP

#include "tinman_navigation.hpp"
#include "can_detection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "manipulation.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

/**
 * @class TinMan
 * @brief Main class controlling the TinMan robot, including navigation, detection, and manipulation subsystems.
 */
class TinMan : public rclcpp::Node {
private:
    int cans_collected; ///< Number of cans collected.
    int cans_trashed; ///< Number of cans disposed.

    std::shared_ptr<RobotNavigation> nav_obj; ///< Navigation subsystem object.
    CanDetection detection_obj; ///< Detection subsystem object.
    Manipulation manipulator_obj; ///< Manipulation subsystem object.

public:
    /**
     * @brief Constructor for the TinMan class.
     */
    TinMan();

    /**
     * @brief Initiates the can collection process.
     */
    void collectCans();

    /**
     * @brief Disposes of the collected cans.
     */
    void disposeCans();

    /**
     * @brief Starts the navigation process.
     */
    void startNavigation();

    void moveToGoal();

    /**
     * @brief Callback function for the image subscriber.
     * @param msg Image message received from the camera.
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_; ///< Publisher for the initial pose.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; ///< Subscriber for the camera feed.
};

#endif // TINMAN_HPP
