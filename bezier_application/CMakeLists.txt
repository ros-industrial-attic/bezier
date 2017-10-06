cmake_minimum_required(VERSION 2.8.3)
project(bezier_application)
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  bezier_library
  fanuc_m20ia_support
  moveit_ros_planning_interface
  roscpp
  rviz_visual_tools
  tf
  tf_conversions
  visualization_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS
  bezier_library
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Surfacing example
add_executable(
  ${PROJECT_NAME}_surfacing
  src/surfacing.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_surfacing
  ${catkin_LIBRARIES}
)

# Painting example
add_executable(
  ${PROJECT_NAME}_painting
  src/painting.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_painting
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_bezier_application.xml)
endif()
