/**
 * @file can_detection.hpp
 * @brief This file contains the declaration of the CanDetection class. The code will use OpenCV and YOLO to detect the cans in the environment.
 * @date November 18, 2024
 * @version 1.0
 * @copyright MIT License 
 * @contributors : 1. Swaraj Mundruppady Rao
 *                 2. Koustubh 
 *                 3. Keyur Borad 
 */

#ifndef CAN_DETECTION_HPP
#define CAN_DETECTION_HPP

#pragma once 

// C++ Standard Libraries
#include <iostream>
#include <vector>
#include <memory>

// OpenCV Libraries
#include <opencv2/core.hpp>       // Core functionalities
#include <opencv2/highgui.hpp>    // High-level GUI functionalities
#include <opencv2/imgproc.hpp>    // Image processing functions
#include <opencv2/dnn.hpp>        // Deep Neural Network module (YOLO integration)

// ROS2 Core Libraries
#include <rclcpp/rclcpp.hpp>      // ROS2 core library
#include <sensor_msgs/msg/image.hpp>         // For raw image messages
#include <sensor_msgs/msg/compressed_image.hpp> // For compressed image messages
#include <sensor_msgs/msg/camera_info.hpp>   // Camera information
#include <geometry_msgs/msg/pose.hpp>        // Geometric poses
#include <geometry_msgs/msg/pose_stamped.hpp> // Stamped poses
#include <nav_msgs/msg/odometry.hpp>         // Odometry information
#include <sensor_msgs/msg/laser_scan.hpp>    // Laser scan data

// CV Bridge (for image conversion between ROS and OpenCV)
#include <cv_bridge/cv_bridge.h>

// Image Transport (for handling ROS image topics)
#include <image_transport/image_transport.hpp>

/**
 * @class CanDetection
 * @brief Class responsible for detecting cans in the environment.
 */
class CanDetection {
    private:
        sensor_msgs::msg::Image camera_frame; ///< Image data from the camera.
        std::vector<std::string> detected_objects; ///< List of detected objects.

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

        /**
         * @brief Sets the camera frame for processing.
         */
        void setCameraFrame(const sensor_msgs::msg::Image& frame);
        

};

#endif  // CAN_DETECTION_HPP
