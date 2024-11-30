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
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
        camera_frame = cv_ptr->image; // Store the converted OpenCV Mat
    } catch (const cv_bridge::Exception& e) {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
    }
}

float CanDetection::trackCentroid() {
    if (camera_frame.empty()) {
        std::cerr << "Camera frame is empty!" << std::endl;
        return -1.0;
    }

    cv::Mat hsv_frame, mask, result;
    // Convert image to HSV
    cv::cvtColor(camera_frame, hsv_frame, cv::COLOR_BGR2HSV);

    // Define HSV range for green color (adjust based on lighting)
    cv::Scalar lower_green(35, 100, 50); // Lower HSV boundary for green
    cv::Scalar upper_green(85, 255, 255); // Upper HSV boundary for green

    // Threshold the HSV image to get only green colors
    cv::inRange(hsv_frame, lower_green, upper_green, mask);

    // Find contours in the binary mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        std::cerr << "No green objects detected!" << std::endl;
        return -1.0;
    }

    // Find the largest contour (assumes the can is the largest green object)
    double max_area = 0;
    int largest_contour_idx = -1;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            largest_contour_idx = static_cast<int>(i);
        }
    }

    if (largest_contour_idx == -1) {
        std::cerr << "Failed to identify the largest green object!" << std::endl;
        return -1.0;
    }

    // Get the bounding box of the largest contour
    cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_idx]);

    // Calculate the centroid of the bounding box
    float centroid_x = bounding_box.x + (bounding_box.width / 2.0);
    float centroid_y = bounding_box.y + (bounding_box.height / 2.0);

    // Draw the bounding box on the original image
    cv::rectangle(camera_frame, bounding_box, cv::Scalar(0, 255, 0), 2); // Green bounding box

    // Annotate the centroid
    cv::circle(camera_frame, cv::Point(centroid_x, centroid_y), 5, cv::Scalar(0, 0, 255), -1); // Red circle

    // Display the result with the bounding box
    cv::imshow("Green Can Detection", camera_frame);
    cv::waitKey(1); // Allow OpenCV to process GUI events

    std::cout << "Centroid of the green can: (" << centroid_x << ", " << centroid_y << ")" << std::endl;

    return centroid_x; // You can also return centroid_y if needed
}
