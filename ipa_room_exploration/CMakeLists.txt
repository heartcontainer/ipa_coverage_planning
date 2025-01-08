cmake_minimum_required(VERSION 3.8)
project(ipa_room_exploration)

# Enable C++17 standard
add_compile_options(-std=c++17)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
# find_package(cob_map_accessibility_analysis REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system chrono thread)
find_package(ipa_building_msgs REQUIRED)
find_package(ipa_building_navigation REQUIRED)

# Optional dependencies (example: GUROBI)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake/")
find_package(GUROBI)
# if Gurobi was found, set a flag to use it
if(GUROBI_FOUND)
  add_definitions(-DGUROBI_FOUND=${GUROBI_FOUND})
  message(STATUS "Gurobi include dirs: ${GUROBI_INCLUDE_DIRS}")
  message(STATUS "Gurobi lib dirs: ${GUROBI_LIBRARIES}")
endif()

# Include directories
include_directories(
  common/include
  ros/include
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GUROBI_INCLUDE_DIRS}
)

### package library
add_library(${PROJECT_NAME} STATIC
  common/src/grid_point_explorator.cpp
  common/src/boustrophedon_explorator.cpp
  common/src/neural_network_explorator.cpp
  common/src/convex_sensor_placement_explorator.cpp
  common/src/energy_functional_explorator.cpp
  common/src/flow_network_explorator.cpp
  common/src/room_rotator.cpp
  common/src/meanshift2d.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  OpenCV
  Boost
  Eigen3
  angles
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
  # cob_map_accessibility_analysis
  ipa_building_msgs
  ipa_building_navigation
)
if(GUROBI_FOUND)
  target_link_libraries(${PROJECT_NAME} ${GUROBI_LIBRARIES})
endif()

# Room exploration action server
# add_executable(room_exploration_server ros/src/room_exploration_action_server.cpp ros/src/fov_to_robot_mapper.cpp)
# ament_target_dependencies(room_exploration_server
#   rclcpp
#   rclcpp_action
#   OpenCV
#   Boost
# )
# target_link_libraries(room_exploration_server ${PROJECT_NAME})

# Coverage check library
# add_library(libcoverage_check_server ros/src/coverage_check_server.cpp)
# ament_target_dependencies(libcoverage_check_server rclcpp OpenCV Boost)

# Coverage check server
# add_executable(coverage_check_server ros/src/coverage_check_server_main.cpp)
# ament_target_dependencies(coverage_check_server rclcpp OpenCV Boost)
# target_link_libraries(coverage_check_server libcoverage_check_server)

# Coverage monitor server
# add_executable(coverage_monitor_server ros/src/coverage_monitor_server.cpp)
# ament_target_dependencies(coverage_monitor_server rclcpp OpenCV Boost)
# target_link_libraries(coverage_monitor_server libcoverage_check_server)

# Room exploration client
# add_executable(room_exploration_client ros/src/room_exploration_action_client.cpp)
# ament_target_dependencies(room_exploration_client rclcpp rclcpp_action OpenCV Boost)

# Room exploration evaluation
# add_executable(room_exploration_evaluation ros/src/room_exploration_evaluation.cpp ros/src/fov_to_robot_mapper.cpp)
# ament_target_dependencies(room_exploration_evaluation rclcpp OpenCV)
# target_link_libraries(room_exploration_evaluation libcoverage_check_server)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  # room_exploration_server
  # room_exploration_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY common/include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ros/include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ros/launch DESTINATION share/${PROJECT_NAME}/ros PATTERN "*.launch" EXCLUDE)

ament_package()
