cmake_minimum_required(VERSION 3.8)
project(canmv_udp_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(canmv_udp_receiver src/canmv_udp_receiver.cpp)
target_compile_features(canmv_udp_receiver PUBLIC cxx_std_17)
ament_target_dependencies(canmv_udp_receiver rclcpp std_msgs)

install(TARGETS canmv_udp_receiver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
