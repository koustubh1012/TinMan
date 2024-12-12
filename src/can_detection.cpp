#include "can_detection.hpp"

CanDetection::CanDetection() {
  // Constructor initialization if necessary
}

CanDetection::~CanDetection() {
  // Destructor cleanup if necessary
}

void CanDetection::setCameraFrame(const sensor_msgs::msg::Image& frame) {
  try {
    // Convert ROS2 Image to OpenCV Mat using cv_bridge
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);

    // Store the converted OpenCV Mat
    camera_frame = cv_ptr->image;  // Store the converted OpenCV Mat
  }

  // Handle exceptions
  catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s",
                 e.what());
  }
}

// Function to track the centroid of detected objects
float CanDetection::trackCentroid() {
  if (camera_frame.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Camera frame is empty!");
    return -1.0;
  }

  // Convert image to HSV
  cv::Mat hsv_frame, mask, result;

  // Convert image to HSV
  cv::cvtColor(camera_frame, hsv_frame, cv::COLOR_BGR2HSV);

  // Define HSV range for green color (adjust based on lighting)

  // Lower and upper bounds for green color in HSV
  cv::Scalar lower_green(35, 100, 50);
  cv::Scalar upper_green(85, 255, 255);

  // Threshold the HSV image to get only green colors
  cv::inRange(hsv_frame, lower_green, upper_green, mask);

  // Noise reduction
  cv::GaussianBlur(mask, mask, cv::Size(5, 5), 0);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1),
                   2);

  // Find contours in the binary mask
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    return -1.0;
  }

  // Find the largest contour (assumes the can is the largest green object)
  double max_area = 0;

  // Initialize the index of the largest contour
  int largest_contour_idx = -1;
  for (size_t i = 0; i < contours.size(); ++i) {
    // Calculate the area of the contour
    double area = cv::contourArea(contours[i]);

    // Update the largest contour if the area is greater than the current max
    if (area > max_area && area > 3000) {
      max_area = area;
      largest_contour_idx = static_cast<int>(i);
    }
  }

  // Check if the contour area exceeds a certain threshold
  if (max_area > 1000000) {
    return -2.0;  // Special value to indicate stop condition
  }

  // Check if the largest contour was found
  if (largest_contour_idx == -1) {
    std::cerr << "Failed to identify the largest green object!" << std::endl;
    return -1.0;
  }

  // Get the bounding box of the largest contour
  cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_idx]);

  // Calculate the centroid of the bounding box
  float centroid_x = bounding_box.x + (bounding_box.width / 2.0);

  // Display the annotated image
  return centroid_x;
}
