cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(cpp_node src/MyFirstNode.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_station src/RobotNewsStation.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(smartphone src/Smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

add_executable(number_publisher src/NumberPublisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/NumberCounter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

add_executable(add_two_ints_server src/AddTwoIntsServer.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client_no_oop src/AddTwoIntsClientNoOop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

add_executable(add_two_ints_client src/AddTwoIntsClient.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(hardware_status_publisher src/HardwareStatusPublisher.cpp)
ament_target_dependencies(hardware_status_publisher rclcpp my_robot_interfaces)

install(TARGETS
  cpp_node
  robot_news_station
  smartphone
  number_publisher
  number_counter
  add_two_ints_server
  add_two_ints_client_no_oop
  add_two_ints_client
  hardware_status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
