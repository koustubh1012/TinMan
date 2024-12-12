#include "tinman.hpp"

TinMan::TinMan()
    : Node("tinman"),
      goal_x(3.9),
      goal_y(5.9),
      tolerance(0.4),
      nav_to_bin(false) {
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&TinMan::imageCallback, this, std::placeholders::_1));

  pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TinMan::poseCallback, this, std::placeholders::_1));

  nav_obj_ = std::make_shared<RobotNavigation>();
  manipulator_obj_ = std::make_shared<Manipulation>();

  RCLCPP_INFO(this->get_logger(), "TinMan node initialized.");
}

geometry_msgs::msg::Pose TinMan::getCurrentPosition() const {
  return current_pose_;
}

void TinMan::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose_ = msg->pose.pose;

  if (nav_to_bin) {
    double distance_to_goal =
        std::sqrt(std::pow(current_pose_.position.x - goal_x, 2) +
                  std::pow(current_pose_.position.y - goal_y, 2));

    RCLCPP_INFO(this->get_logger(), "Distance to trash can: %.2f",
                distance_to_goal);

    if (distance_to_goal <= tolerance) {
      RCLCPP_INFO(this->get_logger(), "Goal reached at x: %.2f, y: %.2f.",
                  goal_x, goal_y);
      manipulator_obj_->dropCan = true;
      RCLCPP_INFO(this->get_logger(), "Dropping the can.");
      nav_to_bin = false;
      manipulator_obj_->spawnCanEntity();
    }
  }
}

void TinMan::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  detector_.setCameraFrame(*msg);
  float centroid_x = detector_.trackCentroid();

  if (centroid_x == -2.0) {  // Robot has stopped
    nav_obj_->navigateToCan(centroid_x);
    manipulator_obj_->deleteCanEntity("green_bin");

    testNavigation(goal_x, goal_y);
    nav_to_bin = true;

    if (manipulator_obj_->dropCan) {
      manipulator_obj_->spawnCanEntity();
      manipulator_obj_->dropCan = false;
    }
    return;
  }

  if (centroid_x >= 0) {
    nav_obj_->navigateToCan(centroid_x);
  }
}

void TinMan::testNavigation(double x, double y) {
  RCLCPP_INFO(this->get_logger(), "Moving to position x: %.2f, y: %.2f", x, y);
  nav_obj_->moveToPosition(x, y);
}
