/**
 * @file tinman.hpp
 * @brief Header file for the TinMan class, the main controller for the robot.
 */

#ifndef TINMAN_HPP
#define TINMAN_HPP

#include "robot_navigation.hpp"
#include "can_detection.hpp"
#include "manipulation.hpp"

/**
 * @class TinMan
 * @brief Main class controlling the TinMan robot, including navigation, detection, and manipulation subsystems.
 */
class TinMan {
private:
    int cans_collected; ///< Number of cans collected.
    int cans_trashed; ///< Number of cans disposed.

    RobotNavigation nav_obj; ///< Navigation subsystem object.
    CanDetection detection_obj; ///< Detection subsystem object.
    Manipulation manipulator_obj; ///< Manipulation subsystem object.

public:
    /**
     * @brief Constructor for the TinMan class.
     */
    TinMan();

    /**
     * @brief Initiates the can collection process.
     */
    void collectCans();

    /**
     * @brief Disposes of the collected cans.
     */
    void disposeCans();

    /**
     * @brief Starts the navigation process.
     */
    void startNavigation();
};

#endif // TINMAN_HPP
