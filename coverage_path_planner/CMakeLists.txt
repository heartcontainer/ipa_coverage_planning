cmake_minimum_required(VERSION 3.5)
project(coverage_path_planner)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_util REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
)

set(executable_name coverage_path_planner)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/coverage_path_planner.cpp
)

set(dependencies
  rclcpp
  rclcpp_components
  nav2_util
  OpenCV
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "coverage_path_planner::CoveragePathPlanner")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
