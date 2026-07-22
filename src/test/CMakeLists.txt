cmake_minimum_required(VERSION 3.8)
project(test_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(test_node
     src/test_node.cpp
     src/position_controller.cpp
)

ament_target_dependencies(test_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  std_msgs
)

target_include_directories(test_node PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)
target_include_directories(test_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
install(TARGETS test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
