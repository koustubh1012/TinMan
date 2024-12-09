#include "manipulation.hpp"
#include <fstream>
#include <ios>
#include <iostream>
#include <sstream>

Manipulation::Manipulation() {
    manip_node_ = rclcpp::Node::make_shared("manipulation_node");
    delete_client_ = manip_node_->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
    spawn_client = manip_node_->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    dropCan=false;
}


void Manipulation::deleteCanEntity(const std::string &entity_name) {
    if (!delete_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(manip_node_->get_logger(), "Service /delete_entity not available.");
        return;
    }

    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = entity_name;

    auto result = delete_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(manip_node_, result, std::chrono::seconds(10)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(manip_node_->get_logger(), "Successfully deleted entity: %s", entity_name.c_str());
    } else {
        RCLCPP_ERROR(manip_node_->get_logger(), "Failed to delete entity: %s", entity_name.c_str());
    }
}


void Manipulation::spawnCanEntity() {

    std::cout << "Spawning can entity..." << std::endl;
    // Wait for the spawn service to be available
    if (!spawn_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(manip_node_->get_logger(), "Service /spawn_entity not available.");
        return;
    }

    // Read the SDF file from the specified path
    std::ifstream file("/home/keyur/enpm700/ROS2/ros2_ws/src/TinMan/models/green_can/model.sdf");
    
    if (!file.is_open()) {
        RCLCPP_ERROR(manip_node_->get_logger(), "Failed to open SDF file: /home/keyur/enpm700/ROS2/ros2_ws/src/TinMan/models/green_can/model.sdf");
        return;
    }

    // Load the contents of the SDF file into a string
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string model_xml = buffer.str();

    // Create the spawn entity request
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    
    // Set the entity name
    request->name = "can";

    // Set the model XML from the file
    request->xml = model_xml;

    // Set the spawn pose (position and orientation)
    geometry_msgs::msg::Pose spawn_pose;
    spawn_pose.position.x = 3.5;
    spawn_pose.position.y = 3.0;
    spawn_pose.position.z = 0.0;  // Spawn on the ground (Z = 0)
    spawn_pose.orientation.w = 1.0;  // No rotation (quaternion)

    request->initial_pose = spawn_pose;

    // Send the spawn request and wait for the result
    auto result = spawn_client->async_send_request(request);

    // Wait for the result and handle success/failure
    if (rclcpp::spin_until_future_complete(manip_node_, result, std::chrono::seconds(10)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(manip_node_->get_logger(), "Successfully spawned can at (3.5, 3.0, 0).");
    } else {
        RCLCPP_ERROR(manip_node_->get_logger(), "Failed to spawn can.");
    }
}


void Manipulation::pickUpCan() {
    // Stub for picking up a can
}
