#include "manipulation.hpp"


Manipulation::Manipulation() {

    // Create a node for manipulation tasks
    manip_node_ = rclcpp::Node::make_shared("manipulation_node");

    // Create clients for the delete and spawn services
    delete_client_ = manip_node_->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");

    // Create a client for the spawn service
    spawn_client = manip_node_->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    // Initialize the dropCan flag
    dropCan=false;
}


void Manipulation::deleteCanEntity(const std::string &entity_name) {

    // Wait for the delete service to be available
    if (!delete_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(manip_node_->get_logger(), "Service /delete_entity not available.");
        return;
    }

    // Create the delete entity request
    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    // Set the entity name to be deleted
    request->name = entity_name;

    // Send the delete request and wait for the result
    auto result = delete_client_->async_send_request(request);

    // Wait for the result and handle success/failure
    if (rclcpp::spin_until_future_complete(manip_node_, result, std::chrono::seconds(10)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(manip_node_->get_logger(), "Successfully deleted entity: %s", entity_name.c_str());
    } else {
        RCLCPP_ERROR(manip_node_->get_logger(), "Failed to delete entity: %s", entity_name.c_str());
    }
}

// Function to spawn a can entity in the simulation
void Manipulation::spawnCanEntity() {
    RCLCPP_INFO(manip_node_->get_logger(), "Spawning can entity...");
    
    // Wait for the spawn service to be available
    if (!spawn_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(manip_node_->get_logger(), "Service /spawn_entity not available.");
        return;
    }

    // Read the SDF file from the specified path
    std::string package_path = ament_index_cpp::get_package_share_directory("tinman");
    std::string file_path = package_path + "/models/green_can/model.sdf";
    std::ifstream file(file_path);
    
    if (!file.is_open()) {
        RCLCPP_ERROR(manip_node_->get_logger(), "Failed to open SDF file");
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
    spawn_pose.position.x = -4.0;
    spawn_pose.position.y = -2.0;
    spawn_pose.position.z = 10.0;
    spawn_pose.orientation.w = 1.0;

    // Set the initial pose of the entity
    request->initial_pose = spawn_pose;

    // Send the spawn request and wait for the result
    auto result = spawn_client->async_send_request(request);

    // Wait for the result and handle success/failure
    if (rclcpp::spin_until_future_complete(manip_node_, result, std::chrono::seconds(10)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(manip_node_->get_logger(), "Successfully spawned can at desired location.");
    } else {
        RCLCPP_ERROR(manip_node_->get_logger(), "Failed to spawn can.");
    }
}

