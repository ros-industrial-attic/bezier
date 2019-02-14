cmake_minimum_required(VERSION 3.2)
project(bezier)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz_visual_tools
)

find_package(PCL 1.8.0 REQUIRED)

find_package(VTK 7.1.0 QUIET)
if (NOT VTK_FOUND)
  find_package(VTK 8.0.0 REQUIRED)
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  ${PROJECT_NAME}_painting
  ${PROJECT_NAME}_surfacing
  ${PROJECT_NAME}_tire_machining
  CATKIN_DEPENDS
  roscpp
  rviz_visual_tools
  DEPENDS
  VTK
)

###########
## Build ##
###########

include_directories(
  include
  ${VTK_USE_FILE}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# Bezier library
add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}_library.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  ${VTK_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Bezier surfacing
add_library(
  ${PROJECT_NAME}_surfacing
  src/surfacing.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_surfacing
  ${PROJECT_NAME}
  ${VTK_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_surfacing
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Bezier painting
add_library(
  ${PROJECT_NAME}_painting
  src/painting.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_painting
  ${PROJECT_NAME}
  ${VTK_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_painting
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Tire machining
add_library(
  ${PROJECT_NAME}_tire_machining
  src/tire_machining.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_tire_machining
  ${PROJECT_NAME}
  ${VTK_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_tire_machining
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_painting
  ${PROJECT_NAME}_surfacing
  ${PROJECT_NAME}_tire_machining
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION
  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
  meshes
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_custom_target(
  ${PROJECT_NAME}_doc
  rosdoc_lite -o ${CATKIN_DEVEL_PREFIX}/doc/${PROJECT_NAME} ${CATKIN_DEVEL_PREFIX}/../src/${PROJECT_NAME}
)
