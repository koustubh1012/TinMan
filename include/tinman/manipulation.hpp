/**
 * @file manipulation.hpp
 * @brief Header file for the Manipulation class, handling physical interactions
 * with objects.
 * @date Novemeber 18 2024
 * @version 1.0
 * @copyright MIT License
 * @contributors : 1. Swaraj Mundruppady Rao
 *                 2. Koustubh
 *                 3. Keyur Borad
 */

#ifndef MANIPULATION_HPP
#define MANIPULATION_HPP

#pragma once

// ROS2 headers
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

// cpp headers
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <ios>
#include <sstream>

/**
 * @class Manipulation
 * @brief Class responsible for manipulation tasks like picking up and dropping
 * cans.
 */
class Manipulation {
 public:
  /**
   * @brief Constructor for the Manipulation class.
   */
  Manipulation();

  /**
   * @brief Deletes a can entity from the simulation.
   */
  void deleteCanEntity(const std::string& entity_name);

  /**
   * @brief Spawns a can entity in the simulation.
   */
  void spawnCanEntity();

  bool dropCan;

 private:
  // Node for manipulation tasks
  rclcpp::Node::SharedPtr manip_node_;

  // Clients for the delete and spawn services
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;

  // Client for the spawn service
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client;
};
#endif  // MANIPULATION_HPP
