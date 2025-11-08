cmake_minimum_required(VERSION 3.8)
project(battery_led_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(batery_led_interfaces REQUIRED)

add_executable(battery_led_node src/battery_node_led.cpp)
ament_target_dependencies(battery_led_node rclcpp batery_led_interfaces)

add_executable(led_node_panel src/led_node_panel.cpp)
ament_target_dependencies(led_node_panel rclcpp batery_led_interfaces)

install(TARGETS 
  battery_led_node
  led_node_panel
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
