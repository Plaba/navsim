cmake_minimum_required(VERSION 3.5)

project(nav_testbench)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2)
find_package(tf2_ros)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  tf2_sensor_msgs
  PCL
  pcl_conversions
)

include_directories(include ${Boost_INCLUDE_DIR})

add_library(nav_testbench_utils src/utils/simulation_environment.cpp src/fake_sensor_node.cpp)

add_executable(fake_robot_odom src/fake_robot_odom.cpp)
add_executable(fake_lidar src/fake_lidar.cpp)
add_executable(fake_lane_camera src/fake_lane_camera.cpp)

target_link_libraries(fake_lidar ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} nav_testbench_utils)
target_link_libraries(fake_lane_camera ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} nav_testbench_utils)

install(DIRECTORY include DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

find_package(Boost REQUIRED COMPONENTS system)

ament_target_dependencies(nav_testbench_utils ${DEPENDENCIES})
ament_target_dependencies(fake_robot_odom ${DEPENDENCIES})
ament_target_dependencies(fake_lidar ${DEPENDENCIES})
ament_target_dependencies(fake_lane_camera ${DEPENDENCIES})

install(TARGETS fake_lidar fake_lane_camera fake_robot_odom DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
