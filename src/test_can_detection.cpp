#include "rclcpp/rclcpp.hpp"
#include "can_detection.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "tinman_navigation.hpp"
#include "manipulation.hpp"
#include <cmath>



class CanDetectionTester : public rclcpp::Node {
public:
    CanDetectionTester() : Node("can_detection_tester") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&CanDetectionTester::imageCallback, this, std::placeholders::_1));
        pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                                "/odom", 10, std::bind(&CanDetectionTester::poseCallback, this, std::placeholders::_1));

        nav_obj_ = std::make_shared<RobotNavigation>();
        manipulator_obj_ = std::make_shared<Manipulation>();


        RCLCPP_INFO(this->get_logger(), "CanDetectionTester initialized.");
    
        // testNavigation(3.0, 3.0);

    }


private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    CanDetection detector_;
    std::shared_ptr<RobotNavigation> nav_obj_;
    std::shared_ptr<Manipulation> manipulator_obj_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;
    geometry_msgs::msg::Pose current_pose_; // To store the current position
    double goal_x = 3.0;
    double goal_y = 3.0;
    const double tolerance = 0.3;
    bool nav_to_bin = false;

    geometry_msgs::msg::Pose getCurrentPosition() const {
    return current_pose_;
    }
    
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        // RCLCPP_INFO(this->get_logger(), "Current position updated: x=%.2f, y=%.2f",
        //             current_pose_.position.x, current_pose_.position.y);

        if (nav_to_bin){
            double distance_to_goal = std::sqrt(
            std::pow(current_pose_.position.x - goal_x, 2) +
            std::pow(current_pose_.position.y - goal_y, 2));
            std::cout <<"DISTANCE TO TRASH CAN: " << distance_to_goal << std::endl;

            if (distance_to_goal <= tolerance){
                RCLCPP_INFO(this->get_logger(), "Goal reached at position x: %.2f, y: %.2f (current position x: %.2f, y: %.2f)",
                    goal_x, goal_y, current_pose_.position.x, current_pose_.position.y);
                manipulator_obj_->dropCan = true;
                std::cout <<"DROPPING THE CAN" << std::endl;
                nav_to_bin = false;
                manipulator_obj_->dropCan = true;
                manipulator_obj_->spawnCanEntity();
            }
        }

        }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        detector_.setCameraFrame(*msg);
        float centroid_x = detector_.trackCentroid();

        if (centroid_x == -2.0) { // Robot has stopped
            // RCLCPP_INFO(this->get_logger(), "Stopping robot and deleting green can.");

            nav_obj_->navigateToCan(centroid_x);
            manipulator_obj_->deleteCanEntity("green_bin");
            

            testNavigation(goal_x, goal_y);
            nav_to_bin = true;
            if(manipulator_obj_->dropCan){
                manipulator_obj_->spawnCanEntity();
                manipulator_obj_->dropCan = false;
            }

            return;
        }

        if (centroid_x >= 0) {
            // RCLCPP_INFO(this->get_logger(), "Green can detected at centroid x: %.2f", centroid_x);
            nav_obj_->navigateToCan(centroid_x);
        } else {
            // RCLCPP_WARN(this->get_logger(), "No green can detected.");
        }
    }

    void testNavigation(double x, double y) {
        RCLCPP_INFO(this->get_logger(), "Moving to position x: %.2f, y: %.2f", x, y);
        nav_obj_->moveToPosition(x, y);

        // // Continuously check if the robot has reached the goal
        // rclcpp::Rate loop_rate(1);  // Check at 1 Hz
        // while (rclcpp::ok()) {
        //     geometry_msgs::msg::Pose current_position = current_pose_;//getCurrentPosition();

        //     double distance_to_goal = std::sqrt(
        //         std::pow(current_position.position.x - x, 2) +
        //         std::pow(current_position.position.y - y, 2)
        //     );

        //     if (distance_to_goal <= tolerance) {
        //         std::cout << distance_to_goal << std::endl;
        //         RCLCPP_INFO(this->get_logger(),
        //                     "Goal reached at position x: %.2f, y: %.2f (current position x: %.2f, y: %.2f)",
        //                     x, y, current_position.position.x, current_position.position.y);

        //         manipulator_obj_->dropCan = true;
        //         break;
        //     } else {
        //         RCLCPP_INFO(this->get_logger(),
        //                     "Current position: x: %.2f, y: %.2f, Distance to goal: %.2f",
        //                     current_position.position.x, current_position.position.y, distance_to_goal);
        //     }

        //     loop_rate.sleep();  // Wait for the next check
        // }
    }



};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanDetectionTester>());
    rclcpp::shutdown();
    return 0;
}