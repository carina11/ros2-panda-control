cmake_minimum_required(VERSION 3.8)
project(panda_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs)
find_package(moveit_ros_planning_interface)
find_package(panda_control_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


add_executable(move_to_pose_server src/move_to_pose_server.cpp)
ament_target_dependencies(move_to_pose_server rclcpp std_msgs moveit_ros_planning_interface geometry_msgs panda_control_msgs)
add_executable(bt_main src/bt_main.cpp src/bt_nodes/move_arm_action.cpp src/bt_nodes/sleep_action.cpp src/bt_nodes/gripper_action.cpp)
ament_target_dependencies(bt_main rclcpp moveit_ros_planning_interface behaviortree_cpp_v3 panda_control_msgs)  # panda_control_msgs hinzufügen
target_link_libraries(bt_main BT::behaviortree_cpp_v3)

install(TARGETS
  move_to_pose_server
  bt_main
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  panda_control/virtual_joy_to_move_command.py
  panda_control/backend.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY bt/ DESTINATION share/${PROJECT_NAME}/bt/)
# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()