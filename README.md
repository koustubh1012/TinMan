# TinMan

## Badges

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![codecov](https://codecov.io/gh/koustubh1012/TinMan/graph/badge.svg?token=XTWB4FQJ5K)](https://codecov.io/gh/koustubh1012/TinMan)
![CICD Workflow status](https://github.com/koustubh1012/TinMan/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)

## Acme Robotics (Tinman) A Mobile Autonomous Robot for Collection of Waste in Cafeteria

## Overview

TinMan is an innovative autonomous robot designed to revolutionize waste management in cafeteria settings by collecting discarded aluminum cans. Developed by ACME Robotics, TinMan is an efficient, hygienic, and sustainable solution that reduces labor costs, promotes recycling, and addresses the challenges of dynamic and cluttered environments.

## Authors

1. FNU Koustubh             (Graduate Student, UMD, <koustubh@umd.edu>)
2. Keyur Borad              (Graduate Student, UMD, <kborad@umd.edu>)
3. Swaraj Mundruppady Rao   (Graduate Student, UMD, <swarajmr@umd.edu>)

## Purpose

Cafeterias generate significant amounts of recyclable waste, particularly aluminum cans. Manual collection is inefficient, time-consuming, and unhygienic. TinMan autonomously navigates the cafeteria, identifies and collects scattered cans, and disposes of them at a designated area once its onboard collection bin is full.

## Product Backlog

For detailed project backlog document, refer to the following document: [Project backlog Document](https://docs.google.com/spreadsheets/d/15zRh9hyb8FhVGP8c3GUeDephmkg42Pm05zHLUwGk4No/edit?gid=0#gid=0)

## Sprint Planning Notes

For detailed sprint planning notes, refer to the following document: [Sprint Planning Notes](https://docs.google.com/document/d/1aYkBTQEc9sz_2KH6B-emRayM40f5MsLR75mS5LxTISY/edit?tab=t.0#heading=h.6j90akwcnl1i)

## Quad Chart

For detailed quad chart, refer to the following presentation: [Quad Chart](https://docs.google.com/presentation/d/1e9iCOqxLyKkk5ClS3eGCo465Qvf01dGejnwnA9A_B3Q/edit#slide=id.g316a9aed667_2_89)

## Developer Documentation

### Dependencies

1. ROS2 Humble: ROS2 Humble installed on a system with Ubuntu 22.04. Follow the instructions on this [website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS2 Humble.

2. OpenCV: Follow the instructions on this [website](https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/) to install OpenCV. This is required to detect the cans using classical image processing algorithms

3. [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu) : Gazebo latest version was installed for deploying simulation environments and running the models in the project package. [website]

## Cloning and Running the Simulation

### Building the code

Before running any of the following, ensure that you are in the main working directory (Root Folder of the directory). To build the project, execute the following commands

```bash
    # Create a working directory:
    mkdir -p ~/ros_ws/src
    # got ho src directory
    cd ~/ros_ws/src
    # Clone the git repository in the src directory
    git clone https://github.com/koustubh1012/TinMan
    # now go back to ros_ws directory and download all the dependencies
    cd .. 
    rosdep install -i --from-path src --rosdistro humble -y
    # build the package
    colcon build --packages-select tinman
    # source underlay and overlay
    source /opt/ros/humble/setup.sh
    source install/setup.bash
```

### Running the Simulation

To setup and launch the Gazebo world, run the following command:

```bash
    ros2 launch tinman cafeteria_world.launch.py
```

You should see a simulation world open with a cafteria model, turtlebot and a green can. This green can will always spawen at a random position.

In another terminal, source the underlay and overlay and run the following command

```bash
    ros2 launch tinman tinman.launch.py
```

### Code Coverage report

To generate the code coverage report, run the following command

```bash
    rm -rf build/ install/
    colcon build --cmake-args -DCOVERAGE=1 
    source install/setup.bash
    colcon test
    ros2 run tinman generate_coverage_report.bash
    open build/tinman/test_coverage/index.html 
```

### To run the tests

Test Driven Development process was followed and the unit and level 2 integration tests can be run by the following command:

```bash
    colcon test --packages-select tinman
    cat log/latest_test/tinman/stdout.log 
```

### Generate doxygen documentation

To generate the doxygen documentation, run the following commands

```bash
    cd ros_ws/src/TinMan
    ./do-docs.bash
    # To open the documentation
    open build/tinman/test_coverage/index.html 
```

## Clang-Formating

```bash
cd ~/ros_ws
#Clang-format 
clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -v "/build/")
```

## Cpp-Lint

```bash
# go to your ros2 workspace directory
cd ~/ros_ws/src
#Cpp Lint
cpplint --filter=-legal/copyright,-build/c++11,+build/c++17,-build/namespaces,-build/include_order $(find . -name *.cpp | grep -v "/build/")
```

## Clang-tidy

```bash
cd ~/ros_ws
# Build the workspace again with the camake args to generate compile_commands.jason file for Clang-tidy to work
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#Clang-tidy command
clang-tidy -p build/tinman --extra-arg=-std=c++17 src/TinMan/src/*.cpp -header-filter=.*
```

## Known Issues / bugs

1. There was an issue before of not executing the level 2 integration test. This was mainly due to the name conflict of package and executable. They should not be same.
2. The turtlebot does not recognise the can sometimes, if it's too far away from it's position.
3. The code coverage build fails because of codecoverage library not found.
4. Ingnition gazebo models fail to load.
5. Colcon test failed soemtimes because the test node "tinman" does not initialise in time
6. The Nav2 navigation stack sometimes sticks and the robot's trajectory is not updated in real time, which may lead to collissions.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Disclaimer

This software is provided "as is," without any warranties or conditions, express or implied. By using this software, you acknowledge that Acme Robotics is not liable for any damages or issues arising from its use. Users are responsible for ensuring the software’s suitability and safety for their specific applications, especially in environments with humans.
