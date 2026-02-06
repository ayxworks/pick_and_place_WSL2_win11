cmake_minimum_required(VERSION 3.8)
project(camera_positioning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(diamond_generator src/diamond_generator.cpp)
ament_target_dependencies(diamond_generator OpenCV)

add_executable(diamond_detector src/diamond_detector_node.cpp)
ament_target_dependencies(diamond_detector
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  OpenCV
  cv_bridge
  image_transport
  geometry_msgs
)

install(TARGETS
  diamond_generator
  diamond_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Other folders
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
