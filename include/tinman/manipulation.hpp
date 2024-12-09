/**
 * @file manipulation.hpp
 * @brief Header file for the Manipulation class, handling physical interactions with objects.
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

#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "geometry_msgs/msg/pose.hpp"


/**
 * @class Manipulation
 * @brief Class responsible for manipulation tasks like picking up and dropping cans.
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

    // /**
    //  * @brief Drops a can into the collection bin.
    //  */
    // bool dropCan();

    /**
     * @brief Picks up a detected can.
     */
    void pickUpCan();

    bool dropCan;




private:
    rclcpp::Node::SharedPtr manip_node_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client;
    
};

#endif // MANIPULATION_HPP
