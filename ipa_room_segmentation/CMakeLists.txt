cmake_minimum_required(VERSION 3.8)
project(ipa_room_segmentation)

# build with c++11
add_compile_options(-std=c++17)

# Set dependencies
set(RUN_PACKAGES
  cv_bridge
  ipa_building_msgs
  nav_msgs
  rosidl_default_runtime
  rclcpp
  rclcpp_action
  sensor_msgs
  opengm
)

# set(BUILD_PACKAGES
#   ${RUN_PACKAGES}
# )

find_package(cv_bridge REQUIRED)
find_package(ipa_building_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dlib REQUIRED)
find_package(opengm REQUIRED)

# Check for OpenMP
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

# find_package(${RUN_PACKAGES} REQUIRED)

###################################
## Generate dynamic reconfigure ##
###################################
# Dynamic reconfigure equivalent
# find_package(dynamic_reconfigure REQUIRED)

###################################
## Declare ament specific config ##
###################################

# ament_export_dependencies(${RUN_PACKAGES})

###########
## Build ##
###########
# Include directories
include_directories(
  common/include
  ros/include
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add library for segmentation algorithms
add_library(${PROJECT_NAME} STATIC
  common/src/distance_segmentation.cpp
  common/src/morphological_segmentation.cpp
  common/src/abstract_voronoi_segmentation.cpp
  common/src/voronoi_segmentation.cpp
  common/src/adaboost_classifier.cpp
  common/src/wavefront_region_growing.cpp
  common/src/contains.cpp
  common/src/features.cpp
  common/src/raycasting.cpp
  common/src/meanshift2d.cpp
  common/src/room_class.cpp
  common/src/voronoi_random_field_segmentation.cpp
  common/src/clique_class.cpp
  common/src/cv_boost_loader.cpp
  common/src/voronoi_random_field_features.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${RUN_PACKAGES} Boost OpenCV)
target_link_libraries(${PROJECT_NAME} ${OpenMP_LIBS})

# Add executables
add_executable(room_segmentation_server ros/src/room_segmentation_server.cpp)
ament_target_dependencies(room_segmentation_server ${RUN_PACKAGES})
target_compile_options(room_segmentation_server PRIVATE ${OpenMP_FLAGS})
target_link_libraries(room_segmentation_server ${PROJECT_NAME} ${OpenMP_LIBS} dlib::dlib ${Boost_FILESYSTEM_LIBRARY})

add_executable(room_segmentation_client ros/src/room_segmentation_client.cpp)
ament_target_dependencies(room_segmentation_client ${RUN_PACKAGES})
target_link_libraries(room_segmentation_client ${PROJECT_NAME})

# add_executable(evaluation
#   ros/src/evaluation_numeric_properties.cpp
#   common/src/evaluation_segmentation.cpp)
# ament_target_dependencies(evaluation ${RUN_PACKAGES})
# target_link_libraries(evaluation ${PROJECT_NAME})

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  room_segmentation_server
  room_segmentation_client
  # evaluation
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Install launch and yaml files
install(DIRECTORY ros/launch
  DESTINATION share/${PROJECT_NAME}/ros
  FILES_MATCHING PATTERN "*.launch"
)
