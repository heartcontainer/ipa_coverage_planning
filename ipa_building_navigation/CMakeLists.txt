cmake_minimum_required(VERSION 3.8)
project(ipa_building_navigation)

add_compile_options(-std=c++17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread chrono)
find_package(ipa_building_msgs REQUIRED)

# Declare include directories
include_directories(
  common/include
  ros/include
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# TSP library
add_library(tsp_solvers SHARED
  common/src/A_star_pathplanner.cpp
  common/src/node.cpp
  common/src/nearest_neighbor_TSP.cpp
  common/src/genetic_TSP.cpp
  common/src/concorde_TSP.cpp
)
ament_target_dependencies(tsp_solvers
  rclcpp
  OpenCV
  Boost
)

# Package library
add_library(${PROJECT_NAME} STATIC
  common/src/maximal_clique_finder.cpp
  common/src/set_cover_solver.cpp
  common/src/trolley_position_finder.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  OpenCV
  Boost
  cv_bridge
  visualization_msgs
  ipa_building_msgs
)

# Action server
add_executable(room_sequence_planning_server
  ros/src/room_sequence_planning_action_server.cpp
)
target_link_libraries(room_sequence_planning_server
  tsp_solvers
  ${PROJECT_NAME}
)
ament_target_dependencies(room_sequence_planning_server
  rclcpp
  rclcpp_action
  OpenCV
  Boost
)

# Action client
add_executable(room_sequence_planning_client
  ros/src/room_sequence_planning_action_client.cpp
)
ament_target_dependencies(room_sequence_planning_client
  rclcpp
  rclcpp_action
  OpenCV
  cv_bridge
  ipa_building_msgs
)

# Evaluation
# add_executable(room_sequence_planning_evaluation
#   ros/src/room_sequence_planning_evaluation.cpp
#   common/src/A_star_pathplanner.cpp
#   common/src/node.cpp
# )
# ament_target_dependencies(room_sequence_planning_evaluation
#   rclcpp
#   OpenCV
# )

add_executable(TSP_evaluation
  ros/src/boosttest.cpp
)
target_link_libraries(TSP_evaluation
  tsp_solvers
)
ament_target_dependencies(TSP_evaluation
  rclcpp
  OpenCV
)

# Install targets
install(
  TARGETS ${PROJECT_NAME} tsp_solvers room_sequence_planning_server room_sequence_planning_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directories
install(DIRECTORY common/include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY ros/include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Install launch files
install(DIRECTORY ros/launch DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} tsp_solvers)
ament_package()
