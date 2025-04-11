cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

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

add_executable(my_cpp_node src/my_first_node.cpp)
add_executable(robot_pub_node src/robot_station_pub.cpp)
add_executable(robot_sub_node src/robot_station_sub.cpp)
add_executable(num_pub_node src/number_publisher.cpp)
add_executable(num_sub_node src/number_subscriber.cpp)
add_executable(add_two_ints_server_node src/add_two_ints_server.cpp)
add_executable(add_two_ints_client_node src/add_two_ints_client.cpp)
add_executable(hardware_status_node src/hardware_status.cpp)

ament_target_dependencies(my_cpp_node rclcpp)
ament_target_dependencies(robot_pub_node rclcpp std_msgs)
ament_target_dependencies(robot_sub_node rclcpp std_msgs)
ament_target_dependencies(num_pub_node rclcpp std_msgs )
ament_target_dependencies(num_sub_node rclcpp std_msgs )
ament_target_dependencies(add_two_ints_server_node rclcpp std_msgs example_interfaces)
ament_target_dependencies(add_two_ints_client_node rclcpp std_msgs example_interfaces)
ament_target_dependencies(hardware_status_node rclcpp std_msgs example_interfaces my_robot_interfaces)

install(TARGETS
  my_cpp_node
  robot_pub_node
  robot_sub_node
  num_pub_node
  num_sub_node
  add_two_ints_server_node
  add_two_ints_client_node
  hardware_status_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
