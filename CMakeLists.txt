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
find_package(ament_index_cpp REQUIRED)


###################################################
# Build level 2 unit test (i.e, integration test)
###################################################
# below sets up code coverage flags
if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
endif()


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_FILE test/test.cpp
                src/tinman_navigation.cpp
                src/can_detection.cpp
                src/manipulation.cpp)

  #Add Gtest
  ament_add_gtest(${PROJECT_NAME}_test ${TEST_FILE})
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs geometry_msgs sensor_msgs image_transport cv_bridge OpenCV nav_msgs gazebo_msgs rclcpp_action nav2_msgs ament_index_cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC include/tinman)

  target_compile_definitions(${PROJECT_NAME}_test PUBLIC "TEST_SRC_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/\"")

  install(TARGETS ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME})
  # ament_lint_auto_find_test_dependencies()
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")


###################################################
# Doxygen Rules
# ref: https://cmake.org/cmake/help/latest/module/FindDoxygen.html
###################################################
if (DOXYGEN_FOUND)
  set( DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_EXTRACT_ALL          YES )
  set( DOXYGEN_CLASS_DIAGRAMS       YES )
  set( DOXYGEN_HIDE_UNDOC_RELATIONS NO )
  set( DOXYGEN_HAVE_DOT             YES )
  set( DOXYGEN_CLASS_GRAPH          YES )
  set( DOXYGEN_CALL_GRAPH           YES )
  set( DOXYGEN_CALLER_GRAPH         YES )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_BUILTIN_STL_SUPPORT  YES )
  set( DOXYGEN_EXTRACT_PRIVATE      YES )
  set( DOXYGEN_EXTRACT_PACKAGE      YES )
  set( DOXYGEN_EXTRACT_STATIC       YES )
  set( DOXYGEN_EXTRACT_LOCALMETHODS YES )
  set( DOXYGEN_UML_LOOK             YES )
  set( DOXYGEN_UML_LIMIT_NUM_FIELDS 50 )
  set( DOXYGEN_TEMPLATE_RELATIONS   YES )
  set( DOXYGEN_DOT_GRAPH_MAX_NODES  100 )
  set( DOXYGEN_MAX_DOT_GRAPH_DEPTH  0 )
  set( DOXYGEN_DOT_TRANSPARENT      YES )
else()
  message( FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation" )
endif()

###################################
# Specify Targets 
###################################
# create a target to build documentation
doxygen_add_docs(docs           # target name
  # List of files or directories
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/test)


add_executable(tinman_exec src/tinman.cpp
                      src/tinman_navigation.cpp
                      src/can_detection.cpp
                      src/manipulation.cpp
                      src/main.cpp)


ament_target_dependencies(tinman_exec rclcpp std_msgs geometry_msgs sensor_msgs image_transport cv_bridge OpenCV nav_msgs gazebo_msgs rclcpp_action nav2_msgs)

add_executable(level2_integration_test test/level2_integration_test.cpp)
ament_target_dependencies(level2_integration_test rclcpp std_msgs geometry_msgs gazebo_msgs sensor_msgs image_transport cv_bridge OpenCV nav_msgs rclcpp_action nav2_msgs ament_index_cpp)
target_link_libraries(level2_integration_test catch_ros2::catch_ros2_with_node_main  )


include_directories(include/tinman)


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

## Install some arbitrary executables
install(PROGRAMS      # want to make the file executable (ie. chmod a+x)
  scripts/run_me_for_fun.bash
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME})

ament_package()