#include "tinman_navigation.hpp"
#include <iostream>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>


RobotNavigation::RobotNavigation() : Node("robot_navigation") {
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    //             "/amcl_pose", 10, std::bind(&RobotNavigation::poseCallback, this, std::placeholders::_1));
    pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                                "/odom", 10, std::bind(&RobotNavigation::poseCallback, this, std::placeholders::_1));

    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
      
}


geometry_msgs::msg::Pose RobotNavigation::generateGoal() {
    // Stub for generating a navigation goal
    return geometry_msgs::msg::Pose();
}
geometry_msgs::msg::Pose RobotNavigation::getCurrentPosition() const {
    return current_pose_;
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

void RobotNavigation::navigateToCan(float centroid_x) {
    auto vel = geometry_msgs::msg::Twist();
    if (centroid_x == -2.0) {
        // Stop the robot
        vel.angular.z = 0.0;
        vel.linear.x = 0.0;
        RCLCPP_INFO(this->get_logger(), "Stopping robot due to large contour area.");
    } else {
        int center = 960; // Center of the image
        int deadband = 50; // Deadband around the center
        float scale_factor = 0.001;

        if (centroid_x < center - deadband) {
            vel.angular.z = scale_factor * (center - centroid_x);
            vel.linear.x = 0.0;
        } else if (centroid_x > center + deadband) {
            vel.angular.z = -scale_factor * (centroid_x - center);
            vel.linear.x = 0.0;
        } else {
            vel.angular.z = 0.0;
            vel.linear.x = 0.2;
        }
    }

    velocity_publisher_->publish(vel);
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


void RobotNavigation::moveToPosition(double x, double y) {
    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    // Create an action client for /navigate_to_pose
    auto action_client = rclcpp_action::create_client<NavigateToPose>(shared_from_this(), "navigate_to_pose");

    // Wait for the action server to be available
    if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server /navigate_to_pose not available.");
        return;
    }

    // Create a goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";  // Use the map frame
    goal_msg.pose.header.stamp = this->get_clock()->now();  // Current time
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.x = -0.0014;
    goal_msg.pose.pose.orientation.y = 0.0030;
    goal_msg.pose.pose.orientation.z = 0.4452;
    goal_msg.pose.pose.orientation.w = 0.8954;

    // Send the goal and wait for result
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(rclcpp::get_logger("RobotNavigation"), "Goal reached successfully.");
        } else {
            RCLCPP_WARN(rclcpp::get_logger("RobotNavigation"), "Failed to reach the goal.");
        }
    };

    action_client->async_send_goal(goal_msg, send_goal_options);
}

void RobotNavigation::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    RCLCPP_INFO(this->get_logger(), "Current position updated: x=%.2f, y=%.2f",
                current_pose_.position.x, current_pose_.position.y);
}