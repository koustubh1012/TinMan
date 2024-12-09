cmake_minimum_required(VERSION 3.8)
project(tinman)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)





add_executable(tinman_exec src/tinman.cpp
                      src/tinman_navigation.cpp
                      src/can_detection.cpp
                      src/manipulation.cpp)


ament_target_dependencies(tinman_exec rclcpp std_msgs geometry_msgs sensor_msgs image_transport cv_bridge OpenCV nav_msgs gazebo_msgs rclcpp_action nav2_msgs)

add_executable(level2_integration_test test/level2_integration_test.cpp)
ament_target_dependencies(level2_integration_test rclcpp std_msgs geometry_msgs gazebo_msgs sensor_msgs image_transport cv_bridge OpenCV nav_msgs rclcpp_action nav2_msgs)
target_link_libraries(level2_integration_test catch_ros2::catch_ros2_with_node_main  )


include_directories(include/tinman)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # find_package(ament_lint_auto REQUIRED)

  #Ski
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)

  set(TEST_FILE test/test.cpp
                src/tinman_navigation.cpp
                src/can_detection.cpp
                src/manipulation.cpp)

  #Add Gtest
  ament_add_gtest(${PROJECT_NAME}_test ${TEST_FILE})
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs geometry_msgs sensor_msgs image_transport cv_bridge OpenCV nav_msgs gazebo_msgs rclcpp_action nav2_msgs)
  target_include_directories(${PROJECT_NAME}_test PUBLIC include/tinman)

  target_compile_definitions(${PROJECT_NAME}_test PUBLIC "TEST_SRC_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/\"")

  install(TARGETS ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME})
  # ament_lint_auto_find_test_dependencies()
endif()

catch_ros2_add_integration_test (Level2_Integration_Test
  LAUNCH_FILE integration_test.launch.yaml  # example in YAML
  )

install(TARGETS
  tinman_exec
  level2_integration_test
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/
  DESTINATION include
)
install(DIRECTORY 
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}/maps)

install(FILES
  worlds/cafeteria.world
  DESTINATION share/${PROJECT_NAME}/worlds
)
#######################

add_executable(test_can_detection src/test_can_detection.cpp
                                  src/can_detection.cpp
                                  src/tinman_navigation.cpp
                                  src/manipulation.cpp)
ament_target_dependencies(test_can_detection rclcpp sensor_msgs image_transport 
                          cv_bridge OpenCV nav_msgs gazebo_msgs rclcpp_action nav2_msgs)
install(TARGETS test_can_detection DESTINATION lib/${PROJECT_NAME})

#################
ament_package()