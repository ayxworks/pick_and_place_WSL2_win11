cmake_minimum_required(VERSION 3.8)
project(cam_flange_support)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Other folders
install(
  DIRECTORY 
  launch
  meshes
  rsc
  rviz
  srdf
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()