#include "tinman_navigation.hpp"
#include <iostream>

RobotNavigation::RobotNavigation() {
    // Constructor stub
}

geometry_msgs::msg::Pose RobotNavigation::generateGoal() {
    // Stub for generating a navigation goal
    return geometry_msgs::msg::Pose();
}

void RobotNavigation::sendNavGoal() {
    // Stub for sending a navigation goal
}

void RobotNavigation::cancelNavGoal() {
    // Stub for cancelling a navigation goal
}

void RobotNavigation::tinmanController() {
    // Stub for controlling the robot
}

void RobotNavigation::navigateToCan(float distance) {
    // Stub for navigating to a can
}

void RobotNavigation::setInitialPose() {
    auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
    initial_pose.header.frame_id = "map";
    initial_pose.pose.pose.position.x = 1.0;
    initial_pose.pose.pose.position.y = 1.0;
    initial_pose.pose.pose.position.z = 0.0;
    initial_pose.pose.pose.orientation.x = 0.0;
    initial_pose.pose.pose.orientation.y = 0.0;
    initial_pose.pose.pose.orientation.z = 0.0;
    initial_pose.pose.pose.orientation.w = 1.0;
    std::cout<<"Setting initial pose to x: "<<initial_pose.pose.pose.position.x<<", y: "<<initial_pose.pose.pose.position.y<<std::endl;
    initial_pose_pub_->publish(initial_pose);
}
