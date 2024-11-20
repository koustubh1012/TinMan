/**
 * @brief Test file
 * @file test.cpp
 */

#include <gtest/gtest.h> 
#include "can_detection.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

/**
 * @brief Test for the CanDetection class
 */


//////////////////////////////////////////
//TEST #1
//////////////////////////////////////////
TEST(CanDetectionTest, testTrackCentroid) {
    // Load the test image
    cv::Mat image = cv::imread(std::string(TEST_SRC_DIR)+"/test2.png"); // Ensure test2.jpg exists in the correct path
    ASSERT_FALSE(image.empty()) << "Error: Unable to load image for testing!";

    // Convert cv::Mat to sensor_msgs::msg::Image
    sensor_msgs::msg::Image image_msg;
    try {
        auto cv_bridge_image = std::make_shared<cv_bridge::CvImage>(std_msgs::msg::Header(), "bgr8", image);
        image_msg = *(cv_bridge_image->toImageMsg()); // Dereference the SharedPtr to get the Image object
    } catch (const cv_bridge::Exception& e) {
        FAIL() << "cv_bridge exception: " << e.what();
    }

    // Create an instance of the CanDetection class
    CanDetection can_detection;

    // Set the camera frame for CanDetection
    can_detection.setCameraFrame(image_msg);

    // Call the trackCentroid function and validate the result
    float result = can_detection.trackCentroid();

    // Validate that the centroid is within an expected range
    EXPECT_NEAR(result, 619.0, 10.0) << "Centroid not detected as expected!";
}


//////////////////////////////////////////
//TEST #2
//////////////////////////////////////////
TEST(CanDetectionTest, testTrackCentroid) {
    // Load the test image
    cv::Mat image = cv::imread(std::string(TEST_SRC_DIR)+"/test3.png"); // Ensure test3.jpg exists in the correct path
    ASSERT_FALSE(image.empty()) << "Error: Unable to load image for testing!";

    // Convert cv::Mat to sensor_msgs::msg::Image
    sensor_msgs::msg::Image image_msg;
    try {
        auto cv_bridge_image = std::make_shared<cv_bridge::CvImage>(std_msgs::msg::Header(), "bgr8", image);
        image_msg = *(cv_bridge_image->toImageMsg()); // Dereference the SharedPtr to get the Image object
    } catch (const cv_bridge::Exception& e) {
        FAIL() << "cv_bridge exception: " << e.what();
    }

    // Create an instance of the CanDetection class
    CanDetection can_detection;

    // Set the camera frame for CanDetection
    can_detection.setCameraFrame(image_msg);

    // Call the trackCentroid function and validate the result
    float result = can_detection.trackCentroid();

    // Validate that the centroid is within an expected range
    EXPECT_NEAR(result, 404.0, 10.0) << "Centroid not detected as expected!";
}


//////////////////////////////////////////
//TEST #3
//////////////////////////////////////////

TEST(CanDetectionTest, testTrackCentroid) {
    // Load the test image
    cv::Mat image = cv::imread(std::string(TEST_SRC_DIR)+"/test4.png"); // Ensure test4.jpg exists in the correct path
    ASSERT_FALSE(image.empty()) << "Error: Unable to load image for testing!";

    // Convert cv::Mat to sensor_msgs::msg::Image
    sensor_msgs::msg::Image image_msg;
    try {
        auto cv_bridge_image = std::make_shared<cv_bridge::CvImage>(std_msgs::msg::Header(), "bgr8", image);
        image_msg = *(cv_bridge_image->toImageMsg()); // Dereference the SharedPtr to get the Image object
    } catch (const cv_bridge::Exception& e) {
        FAIL() << "cv_bridge exception: " << e.what();
    }

    // Create an instance of the CanDetection class
    CanDetection can_detection;

    // Set the camera frame for CanDetection
    can_detection.setCameraFrame(image_msg);

    // Call the trackCentroid function and validate the result
    float result = can_detection.trackCentroid();

    // Validate that the centroid is within an expected range
    EXPECT_EQ(result, -1.0) << "Centroid not detected as expected!";
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
