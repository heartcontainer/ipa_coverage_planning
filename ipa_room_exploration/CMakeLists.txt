cmake_minimum_required(VERSION 3.8)
project(ipa_room_exploration)

add_compile_options(-std=c++17)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
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
find_package(tf2_eigen REQUIRED)
find_package(ipa_building_common REQUIRED)

# Optional dependencies (example: GUROBI)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake/")
find_package(GUROBI)
# if Gurobi was found, set a flag to use it
if(GUROBI_FOUND)
  add_definitions(-DGUROBI_FOUND=${GUROBI_FOUND})
  message(STATUS "Gurobi include dirs: ${GUROBI_INCLUDE_DIRS}")
  message(STATUS "Gurobi lib dirs: ${GUROBI_LIBRARIES}")
endif()

# Coin-Or and Cbc, included as ubuntu system package, without Cmake-module so include pkg-config file
# 	!!! Important note: the order of the libraries when linking your executable is important, if it is wrong you get undefined references !!!
find_package(PkgConfig REQUIRED)
pkg_check_modules(CoinUtils REQUIRED coinutils)# coinutils osi-clp clp osi cbc cgl
pkg_check_modules(OsiClp REQUIRED osi-clp)
pkg_check_modules(Clp REQUIRED clp)
pkg_check_modules(Osi REQUIRED osi)
pkg_check_modules(Cbc-lib REQUIRED cbc)
pkg_check_modules(Cgl REQUIRED cgl)
Message(STATUS "coinutils include dirs: " ${CoinUtils_INCLUDE_DIRS})
Message(STATUS "osi-clp include dirs: " ${OsiClp_INCLUDE_DIRS})
Message(STATUS "clp include dirs: " ${Clp_INCLUDE_DIRS})
Message(STATUS "osi include dirs: " ${Osi_INCLUDE_DIRS})
Message(STATUS "cbc include dirs: " ${Cbc-lib_INCLUDE_DIRS})
Message(STATUS "cgl include dirs: " ${Cgl_INCLUDE_DIRS})

# Include directories
include_directories(
  common/include
  ros/include
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GUROBI_INCLUDE_DIRS}
  ${CoinUtils_INCLUDE_DIRS}
	${OsiClp_INCLUDE_DIRS}
	${Clp_INCLUDE_DIRS}
	${Osi_INCLUDE_DIRS}
	${Cgl_INCLUDE_DIRS}
	${Cbc-lib_INCLUDE_DIRS}
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
  common/src/map_accessibility_analysis.cpp
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
  ipa_building_msgs
  ipa_building_navigation
  cv_bridge
  tf2_eigen
)
if(GUROBI_FOUND)
  target_link_libraries(${PROJECT_NAME} ${GUROBI_LIBRARIES})
endif()

# Coverage check library
add_library(libcoverage_check_server ros/src/coverage_check_server.cpp)
ament_target_dependencies(libcoverage_check_server 
  rclcpp 
  OpenCV 
  Boost 
  cv_bridge 
  ipa_building_msgs
)

# Room exploration action server
add_executable(room_exploration_server ros/src/room_exploration_action_server.cpp ros/src/fov_to_robot_mapper.cpp)
ament_target_dependencies(room_exploration_server
  rclcpp
  rclcpp_action
  OpenCV
  Boost
  CoinUtils
	OsiClp
	Clp
	Osi
	Cgl
	Cbc-lib
  ipa_building_navigation
  nav_msgs
  nav2_msgs
  nav2_util
  ipa_building_common
)
target_link_libraries(room_exploration_server
  ${PROJECT_NAME}
  ${CoinUtils_LIBRARIES}
  ${OsiClp_LIBRARIES}
  ${Clp_LIBRARIES}
  ${Osi_LIBRARIES}
  ${Cgl_LIBRARIES}
  ${Cbc-lib_LIBRARIES}
  ${Boost_LIBRARIES}
  libcoverage_check_server
)

# Coverage check server
add_executable(coverage_check_server ros/src/coverage_check_server_main.cpp)
ament_target_dependencies(coverage_check_server 
  rclcpp 
  OpenCV 
  Boost 
  cv_bridge
  ipa_building_msgs
  ipa_building_common
)
target_link_libraries(coverage_check_server libcoverage_check_server)

# Coverage monitor server
# add_executable(coverage_monitor_server ros/src/coverage_monitor_server.cpp)
# ament_target_dependencies(coverage_monitor_server rclcpp OpenCV Boost)
# target_link_libraries(coverage_monitor_server libcoverage_check_server)

# Room exploration client
add_executable(room_exploration_client ros/src/room_exploration_action_client.cpp)
ament_target_dependencies(room_exploration_client
  rclcpp
  rclcpp_action
  OpenCV
  Boost
  cv_bridge
  nav_msgs
  ipa_building_msgs
  ipa_building_common
  tf2
  tf2_ros
)

# Room exploration evaluation
# add_executable(room_exploration_evaluation ros/src/room_exploration_evaluation.cpp ros/src/fov_to_robot_mapper.cpp)
# ament_target_dependencies(room_exploration_evaluation rclcpp OpenCV)
# target_link_libraries(room_exploration_evaluation libcoverage_check_server)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  room_exploration_server
  room_exploration_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY common/include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ros/include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ros/launch DESTINATION share/${PROJECT_NAME})

ament_package()
