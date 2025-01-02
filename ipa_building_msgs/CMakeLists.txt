cmake_minimum_required(VERSION 3.8)
project(ipa_building_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

# Add action files
set(action_files
  action/MapSegmentation.action
  action/FindRoomSequenceWithCheckpoints.action
  action/RoomExploration.action
)

# Add message files
set(msg_files
  msg/RoomInformation.msg
  msg/RoomSequence.msg
)

# Add service files
set(srv_files
  srv/CheckCoverage.srv
  srv/ExtractAreaMapFromLabeledMap.srv
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    geometry_msgs
    std_msgs
    sensor_msgs
    action_msgs
)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(rosidl_default_runtime)
ament_package()
