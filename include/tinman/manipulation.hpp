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
    void deleteCanEntity();

    /**
     * @brief Spawns a can entity in the simulation.
     */
    void spawnCanEntity();

    /**
     * @brief Drops a can into the collection bin.
     */
    void dropCan();

    /**
     * @brief Picks up a detected can.
     */
    void pickUpCan();
};

#endif // MANIPULATION_HPP
