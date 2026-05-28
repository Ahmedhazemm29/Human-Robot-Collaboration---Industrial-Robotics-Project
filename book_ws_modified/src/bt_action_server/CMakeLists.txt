cmake_minimum_required(VERSION 3.8)
project(bt_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ReachLocation.action"
  DEPENDENCIES std_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  "${PROJECT_NAME}" "rosidl_typesupport_cpp")

include_directories(include)

# ── reach_location_server ─────────────────────────────────────────────────────
add_executable(reach_location_server src/reach_location_server.cpp)
ament_target_dependencies(reach_location_server
  rclcpp rclcpp_action std_msgs geometry_msgs
  moveit_ros_planning_interface moveit_msgs)
target_link_libraries(reach_location_server "${cpp_typesupport_target}")

# ── reach_location_client ─────────────────────────────────────────────────────
add_executable(reach_location_client src/reach_location_client.cpp)
ament_target_dependencies(reach_location_client
  rclcpp rclcpp_action std_msgs geometry_msgs)
target_link_libraries(reach_location_client "${cpp_typesupport_target}")

# ── bt_action ─────────────────────────────────────────────────────────────────
add_executable(bt_action src/bt_action.cpp)
ament_target_dependencies(bt_action
  rclcpp rclcpp_action std_msgs geometry_msgs behaviortree_cpp)
target_link_libraries(bt_action "${cpp_typesupport_target}")

install(TARGETS
  reach_location_server
  reach_location_client
  bt_action
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch trees
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
