cmake_minimum_required(VERSION 3.8)
project(ipa_building_common)

add_compile_options(-std=c++17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Declare include directories
include_directories(
  include
)

# Package library
add_library(${PROJECT_NAME} STATIC
  src/dynamic_reconfigure.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directories
install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
