/**
 * @file can_detection.hpp
 * @brief This file contains the declaration of the CanDetection class. The code will use OpenCV and YOLO to detect the cans in the environment.
 * @date Novemeber 18 2024
 * @version 1.0
 * @copyright MIT License 
 * @contributors : 1. Swaraj Mundruppady Rao
 *                 2. Koustubh 
 *                 3. Keyur Borad 
 */


#ifndef CAN_DETECTION_HPP
#define CAN_DETECTION_HPP

#pragma once 

//C++ Standard Libraries
#include <iostream>
#include <vector>
#include <chrono>
#include <memory>
#include <iomanip>

//Open CV Related Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//ROS2 Related Libraries
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

/**
 * @class CanDetection
 * @brief Class responsible for detecting cans in the environment.
 */
class CanDetection {
    private:
        sensor_msgs::msg::Image camera_frame; ///< Image data from the camera.
        std::vector<cv::string> detected_objects; ///< List of detected objects.

    public:
        /**
         * @brief Constructor for the CanDetection class.
         */
        CanDetection();

        /**
         * @brief Destructor for the CanDetection class.
         */
        ~CanDetection();

        /**
         * @brief Tracks the centroid of detected objects.
         * @return The distance to the centroid
         */
        float trackCentroid();

        /**
         * @brief Retrieves the list of detected objects.
         * @return A list of detected object names.
         */
        std::vector<std::string> getDetectedObjects();
};

#endif  // CAN_DETECTION_HPP