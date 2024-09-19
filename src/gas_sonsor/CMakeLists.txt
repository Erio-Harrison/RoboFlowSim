cmake_minimum_required(VERSION 3.8)
project(gas_sonsor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)

# Add the executable
add_executable(gas_dispersion_node src/gas_dispersion_node.cpp)
ament_target_dependencies(gas_dispersion_node rclcpp geometry_msgs std_msgs)

add_executable(gas_sensor_node src/gas_sensor_node.cpp)
ament_target_dependencies(gas_sensor_node rclcpp geometry_msgs std_msgs)

add_executable(visualization_node src/visualization_node.cpp)
ament_target_dependencies(visualization_node rclcpp std_msgs visualization_msgs interactive_markers tf2)

# Install the executable
install(TARGETS
  gas_dispersion_node
  gas_sensor_node
  visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()