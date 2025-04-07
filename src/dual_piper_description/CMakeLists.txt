cmake_minimum_required(VERSION 3.8)
project(dual_piper_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  launch
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()