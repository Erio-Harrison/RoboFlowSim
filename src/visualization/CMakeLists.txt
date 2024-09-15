cmake_minimum_required(VERSION 3.8)
project(visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 添加 gas_flow_visualizer 可执行文件（如果存在的话）
add_executable(gas_flow_visualizer src/gas_flow_visualizer.cpp)
ament_target_dependencies(gas_flow_visualizer rclcpp visualization_msgs)

# 添加新的 gps_visualizer 可执行文件
add_executable(gps_visualizer src/gps_visualizer.cpp)
ament_target_dependencies(gps_visualizer rclcpp visualization_msgs geometry_msgs)

add_executable(realistic_gas_diffusion src/realistic_gas_diffusion.cpp)
ament_target_dependencies(realistic_gas_diffusion rclcpp visualization_msgs)

add_executable(gas_diffusion_2d src/gas_diffusion_2d.cpp)
ament_target_dependencies(gas_diffusion_2d rclcpp visualization_msgs)

install(TARGETS
  gas_flow_visualizer
  gps_visualizer
  realistic_gas_diffusion
  gas_diffusion_2d
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()