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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(bumper_bot_cpp src/simple_parameter.cpp)

ament_target_dependencies(bumper_bot_cpp rclcpp std_msgs rcl_interfaces)

# Install the executable
install(TARGETS
  bumper_bot_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
