cmake_minimum_required(VERSION 3.8)
project(bumper_bot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(turtlesim REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(simple_parameter_node src/simple_parameter.cpp)
add_executable(simple_turtlesim_kinematics src/simple_turtlesim_kinematics.cpp)

ament_target_dependencies(simple_parameter_node rclcpp std_msgs rcl_interfaces)
ament_target_dependencies(simple_turtlesim_kinematics rclcpp std_msgs turtlesim)

# Install the executable
install(TARGETS
  simple_parameter_node
  simple_turtlesim_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
