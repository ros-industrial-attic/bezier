cmake_minimum_required(VERSION 2.8.3)
project(bezier_application)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS bezier_library
										moveit_ros_planning_interface 
										roscpp 
										tf 
										tf_conversions
										visualization_msgs
										moveit_visual_tools
	     								rviz_visual_tools)
find_package(PCL 1.8.0 REQUIRED)
find_package(VTK 7.0 REQUIRED COMPONENTS vtkFiltersHybrid NO_MODULE)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS bezier_library)

include_directories(${VTK_USE_FILE} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}) # Order matters!
add_definitions(${PCL_DEFINITIONS} ${VTK_DEFINITIONS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable (bezier_application src/bezier_application.cpp)
set_property(TARGET bezier_application APPEND_STRING PROPERTY COMPILE_FLAGS -Wall)
if(VTK_LIBRARIES) # May never happen because we expect VTK 7.0 minimum
  target_link_libraries (bezier_application ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} bezier_library)
else()
  target_link_libraries (bezier_application ${catkin_LIBRARIES} vtkHybrid vtkWidgets ${PCL_LIBRARIES} bezier_library)
endif()

