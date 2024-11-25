#include "can_detection.hpp"

CanDetection::CanDetection() {
    // Constructor stub
}

CanDetection::~CanDetection() {
    // Destructor stub
}

void CanDetection::setCameraFrame(const sensor_msgs::msg::Image& frame) {
    try {
        // Convert ROS2 Image to OpenCV Mat using cv_bridge
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
        imshow("Camera Frame", cv_ptr->image);
        // camera_frame = cv_ptr->image; // Store the converted OpenCV Mat
    } catch (const cv_bridge::Exception& e) {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
    }
}


float CanDetection::trackCentroid() {
    //Detect the centroid of the green can in the image, the can is cylindrical


    return 0.0;
}

std::vector<std::string> CanDetection::getDetectedObjects() {
    // Stub for retrieving detected objects
    return {};
}
